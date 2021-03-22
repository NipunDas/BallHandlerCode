// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
public class SetHood extends CommandBase {

  private double targetAngle;
  
  /** Creates a new SetHood. */
  public SetHood(double angle) {
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
    if (targetAngle > RobotContainer.returnBallHandler().getHoodPosition()) {
      RobotContainer.returnBallHandler().setHoodPower(0.1);
    } else {
      RobotContainer.returnBallHandler().setHoodPower(-0.1);
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
    return Math.abs(targetAngle - RobotContainer.returnBallHandler().getHoodPosition()) < 2;
  }
}
