// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FindPowerCell extends CommandBase {

  private boolean right;
  private double power;

  /** Creates a new FindPowerCell. */
  public FindPowerCell(boolean turnRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance(), RobotContainer.returnBallHandler().getInstance());
    right = turnRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getLimelight().getX() == 0) {
      power = 0.4;
    } else {
      power = 0.02 * Math.abs(RobotContainer.getLimelight().getX());
    }
    if (right) {
      RobotContainer.returnDrive().tankDrive(power, -power);
    } else {
      RobotContainer.returnDrive().tankDrive(-power, power);
    }
    RobotContainer.returnBallHandler().spinIntake(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.getLimelight().getX() != 0 && Math.abs(RobotContainer.getLimelight().getX()) < 2);
  }
}
