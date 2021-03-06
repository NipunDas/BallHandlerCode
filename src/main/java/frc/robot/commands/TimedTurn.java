// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class TimedTurn extends CommandBase {

  private boolean turnRight;
  private double turnTime;
  private double startTime;
  /** Creates a new TimedTurn. */
  public TimedTurn(boolean right, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance(), RobotContainer.returnBallHandler().getInstance());
    turnTime = time;
    turnRight = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turnRight) {
      RobotContainer.returnDrive().tankDrive(0.6, -0.6);
    } else {
      RobotContainer.returnDrive().tankDrive(-0.6, 0.6);
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
    return (Timer.getFPGATimestamp() - startTime) > turnTime;
  }
}
