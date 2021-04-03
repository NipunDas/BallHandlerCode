// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class BallHandler extends SubsystemBase {

  public static BallHandler ballHandler;
  private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intakePort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.feederPort);
  private WPI_TalonSRX flywheel = new WPI_TalonSRX(Constants.flywheelPort);
  private WPI_TalonSRX hood = new WPI_TalonSRX(Constants.hoodPort);

  private double kP = 0.15, kI = 0, kD = 0, kS = 0, kV = 0;

  private double kTicksInRotation = 4096.0;
  PIDController flywheelPID = new PIDController(kP, kI, kD);
  SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);
  private double feederThreshold = 0.1; //threshold for determining when to spin feeder

  //4 hood angles
  public static double[] hoodAngles = {40, 50, 60, 70};

  //shooter speed (tangential speed in m/s)
  private double shooterSpeed = 25;

  //intake power (stored as variable so it can be toggled)
  private double intakePower = 0.45;

  /** Creates a new BallHandler. */
  public BallHandler() {
    intake.configFactoryDefault();
    intake.setInverted(false);
    feeder.configFactoryDefault();
    feeder.setInverted(false);
    flywheel.configFactoryDefault();
    flywheel.setInverted(true);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    hood.configFactoryDefault();
    hood.setInverted(false);
    hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    hood.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    hood.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    hood.setSelectedSensorPosition(0, 0, 10);
  }

  public BallHandler getInstance() {
    if(ballHandler == null) {
      ballHandler = new BallHandler();
    }
    return ballHandler;
  }

  public void spinIntake(double pow) {
    intake.set(ControlMode.PercentOutput, pow);
  }

  public double getFlyWheelSpeed() {
    return -1 * flywheel.getSensorCollection().getPulseWidthVelocity() * 10 * (1.0 / kTicksInRotation) * (2 * Math.PI * 0.0508);
  }

  public boolean flyWheelSpeedCorrect() {
    return Math.abs(flywheelPID.getPositionError()) < feederThreshold;
  }

  public double getHoodPosition() {
    return hood.getSelectedSensorPosition(0) * (1.0 / kTicksInRotation) * (16.0/50.0) * 360.0;
  }

  public void setHoodBrake(boolean brake) {
    if (brake) {
      hood.setNeutralMode(NeutralMode.Brake);
    } else {
      hood.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setHoodPower(double pow) {
    hood.set(ControlMode.PercentOutput, -1 * pow);
  }

  public void spinFeeder(double pow) {
    feeder.set(ControlMode.PercentOutput, pow);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.returnRightJoy().getRawButtonPressed(Constants.intakeToggle)) {
      intakePower *= -1;
    }
    //spinIntake(intakePower);
    SmartDashboard.putNumber("Intake Current", intake.getSupplyCurrent());

    //flywheel constant velocity code
    double feedForwardPower = flywheelFeedforward.calculate(shooterSpeed);  
    double flywheelPower = flywheelPID.calculate(getFlyWheelSpeed(), shooterSpeed) + feedForwardPower;
    //flywheel.set(ControlMode.PercentOutput, flywheelPower);
    SmartDashboard.putNumber("Flywheel Speed", getFlyWheelSpeed());
    SmartDashboard.putNumber("Flywheel Power", flywheelPower);

    //hood data put on smart dashboard
    SmartDashboard.putNumber("Hood Position: ", hood.getSelectedSensorPosition(0));

    //hood test code
    setHoodBrake(false);
    if (RobotContainer.returnRightJoy().getRawButton(8)) {
      setHoodBrake(false);
      setHoodPower(0.2);
    } else if (RobotContainer.returnRightJoy().getRawButton(9)) {
      setHoodBrake(false);
      setHoodPower(-0.2);
    } else {
      setHoodPower(0);
      setHoodBrake(true);
    }

    //ultrasonic data put on smart dashboard (adding 0.63 to get distance from back of shooter to target)
    if (RobotContainer.getUltrasonic().canRead()) {
      SmartDashboard.putNumber("Ultrasonic Distance: ", RobotContainer.getUltrasonic().getDistance() + 0.63);
    }
  }
}
