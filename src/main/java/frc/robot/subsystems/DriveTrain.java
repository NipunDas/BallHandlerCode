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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain drive;
  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(Constants.leftDrivePort);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(Constants.rightDrivePort);
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double kTicksToInches = 0.152 * Math.PI * (1.0/4096.0);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //configuring left and right talons
    leftTalon.configFactoryDefault();
    leftTalon.setInverted(false);
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightTalon.configFactoryDefault();
    rightTalon.setInverted(true);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    resetEncoders();

    //resetting the gyro
    navx.reset();
  }

  public DriveTrain getInstance() {
    if (drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  public void tankDrive(double leftPow, double rightPow) {
    leftTalon.set(ControlMode.PercentOutput, leftPow);
    rightTalon.set(ControlMode.PercentOutput, rightPow);
  }

  public void resetEncoders() {
    leftTalon.setSelectedSensorPosition(0, 0, 10);
    rightTalon.setSelectedSensorPosition(0, 0, 10);
  }

  public double getEncoderDistance() {
    return (leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0)) * -0.5 * kTicksToInches;
  }

  public void resetNavx() {
    navx.reset();
  }

  public double getAngle() {
    return navx.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tankDrive(RobotContainer.returnLeftJoy().getY() * 0.5, RobotContainer.returnRightJoy().getY() * 0.5);
  }
}
