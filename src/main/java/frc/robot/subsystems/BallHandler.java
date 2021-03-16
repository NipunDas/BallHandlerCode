// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BallHandler extends SubsystemBase {

  public static BallHandler ballHandler;
  private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intakePort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.feederPort);
  private WPI_TalonSRX flywheel = new WPI_TalonSRX(Constants.flywheelPort);
  private WPI_TalonSRX hood = new WPI_TalonSRX(Constants.hoodPort);

  /** Creates a new BallHandler. */
  public BallHandler() {
    feeder.configFactoryDefault();
    feeder.setInverted(false);
    flywheel.configFactoryDefault();
    flywheel.setInverted(false);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    hood.configFactoryDefault();
    hood.setInverted(false);
    hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
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
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    spinIntake(0.3);
  }
}
