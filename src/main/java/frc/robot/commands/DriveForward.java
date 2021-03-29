// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class DriveForward extends CommandBase {

  double targetDistance, error, power;
  double kP = 0.75, kI = 0, kD = 0;
  double errorSum = 0;
  double dt, previousTime;
  double previousError, dE, derivative;

  /** Creates a new DriveForward. */
  public DriveForward(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance(), RobotContainer.returnBallHandler().getInstance());
    targetDistance = distance;
    error = targetDistance;
    previousTime = Timer.getFPGATimestamp();
    previousError = error;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.returnDrive().resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetDistance - RobotContainer.returnDrive().getEncoderDistance();
    dt = Timer.getFPGATimestamp() - previousTime;
    previousTime = Timer.getFPGATimestamp();
    dE = error - previousError;
    derivative = (dE/dt);
    errorSum += (error * dt);
    power = (error*kP) + (errorSum*kI) + (derivative*kD);
    RobotContainer.returnDrive().tankDrive(power, power);
    RobotContainer.returnBallHandler().spinIntake(0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnDrive().tankDrive(0, 0);
    RobotContainer.returnDrive().resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(error) <= 0.15) {
      return true;
    }
    return false;
  }
}
