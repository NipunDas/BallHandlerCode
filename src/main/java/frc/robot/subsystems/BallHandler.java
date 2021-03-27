// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;

public class BallHandler extends SubsystemBase {

  public static BallHandler ballHandler;
  private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intakePort);
  //private WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.feederPort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(3);
  //private WPI_TalonSRX flywheel = new WPI_TalonSRX(Constants.flywheelPort);
  private WPI_TalonSRX flywheel = new WPI_TalonSRX(2);
  private WPI_TalonSRX hood = new WPI_TalonSRX(Constants.hoodPort);

  private double kTicksInRotation = 4096.0;
  PIDController flywheelPID = new PIDController(0.2, 0.4, 0);
  private double feederThreshold = 0.1; //threshold for determining when to spin feeder

  //4 hood angles
  public static int[] hoodAngles = {40, 50, 60, 70};

  //shooter speed (tangential speed in m/s)
  private double shooterSpeed = 2;

  //intake power (stored as variable so it can be toggled)
  private double intakePower = 0.3;

  /** Creates a new BallHandler. */
  public BallHandler() {
    intake.configFactoryDefault();
    intake.setInverted(false);
    feeder.configFactoryDefault();
    feeder.setInverted(false);
    flywheel.configFactoryDefault();
    flywheel.setInverted(false);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    hood.configFactoryDefault();
    
    hood.setInverted(false);
    hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
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
    hood.set(ControlMode.PercentOutput, pow);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.returnRightJoy().getRawButton(Constants.intakeToggle)) {
      intakePower *= -1;
    }
    spinIntake(intakePower);

    //flywheel constant velocity code
    double flywheelPower = flywheelPID.calculate(getFlyWheelSpeed(), shooterSpeed);
    flywheel.set(ControlMode.PercentOutput, flywheelPower);
    SmartDashboard.putNumber("Flywheel Speed", getFlyWheelSpeed());
    SmartDashboard.putNumber("Flywheel Power", flywheelPower);

    //controlling feeder based on flywheel velocity error (or joystick button)
    if (Math.abs(flywheelPID.getPositionError()) < feederThreshold || RobotContainer.returnRightJoy().getRawButton(Constants.feederButton)) {
      feeder.set(ControlMode.PercentOutput, 0.2);
    } else {
      feeder.set(ControlMode.PercentOutput, 0);
    }

    //hood data put on smart dashboard
    SmartDashboard.putNumber("Hood Position: ", getHoodPosition());

    //ultrasonic data put on smart dashboard
    if (RobotContainer.getUltrasonic().canRead()) {
      SmartDashboard.putNumber("Ultrasonic Distance: ", RobotContainer.getUltrasonic().getDistance());
    }
  }
}
