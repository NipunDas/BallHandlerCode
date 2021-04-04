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

  //kS = 0.486, kV = 0.367, kA = 0.0904
  private double kP = 0.1, kI = 0, kD = 0, kS = 0.485, kV = 0.34, kA = 0.0904;

  private double kTicksInRotation = 4096.0;
  PIDController flywheelPID = new PIDController(kP, kI, kD);
  SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  private double feederThreshold = 0.1; //threshold for determining when to spin feeder

  //4 hood angles
  public static double[] hoodAngles = {59, 47, 52};

  //shooter speed (tangential speed in m/s)
  private double shooterSpeed = 25;

  //intake power (stored as variable so it can be toggled)
  private double intakePower = 0.45;

  //checking if shooter flywheel needs to be set to full power
  private boolean flywheelFullPower = false;

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
    return 90 + (hood.getSelectedSensorPosition(0) * (1.0 / kTicksInRotation) * (16.0/50.0) * 360.0);
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

  public void setFullPower(boolean fullPower) {
    flywheelFullPower = fullPower;
  }

  public void setFlywheelPower() {
    flywheel.set(ControlMode.PercentOutput, 1);
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
    double flywheelPower = feedForwardPower;
    if (Math.abs(flywheelPID.getPositionError()) < 1) {
      flywheelPower += flywheelPID.calculate(getFlyWheelSpeed(), shooterSpeed);
    }
    if (flywheelFullPower) {
      flywheel.set(ControlMode.PercentOutput, 1);
    } else {
      flywheel.set(ControlMode.PercentOutput, flywheelPower/12.0);
    }
    SmartDashboard.putNumber("Flywheel Speed", getFlyWheelSpeed());
    SmartDashboard.putNumber("Flywheel Power", flywheelPower/12.0);

    //hood data put on smart dashboard
    SmartDashboard.putNumber("Hood Position: ", getHoodPosition());

    //hood test code
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

    if (RobotContainer.returnRightJoy().getRawButton(11)) {
      spinIntake(intakePower);
    } else {
      spinIntake(0);
    }

    if (hood.isFwdLimitSwitchClosed() == 0) {
      hood.setSelectedSensorPosition(0, 0, 10);
    }

    //ultrasonic data put on smart dashboard (adding 0.63 to get distance from back of shooter to target)
    if (RobotContainer.getUltrasonic().canRead()) {
      SmartDashboard.putNumber("Ultrasonic Distance: ", RobotContainer.getUltrasonic().getDistance() + 0.63);
    }
  }
}
