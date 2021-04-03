// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
public class ShootBall extends CommandBase {

  private double targetAngle;
  
  /** Creates a new ShootBall. */
  public ShootBall(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnBallHandler().getInstance());
    targetAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.returnBallHandler().setHoodBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = targetAngle - RobotContainer.returnBallHandler().getHoodPosition();
    if (error > 1) {
      RobotContainer.returnBallHandler().setHoodPower(-0.2);
    } else if (error < -1){
      RobotContainer.returnBallHandler().setHoodPower(0.2);
    } else {
      if (RobotContainer.returnBallHandler().flyWheelSpeedCorrect()) {
        RobotContainer.returnBallHandler().spinFeeder(0.3);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnBallHandler().setHoodPower(0);
    RobotContainer.returnBallHandler().setHoodBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
