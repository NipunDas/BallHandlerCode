// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
public class ShootBall extends CommandBase {

  private double targetAngle;
  private boolean fullPower;
  
  /** Creates a new ShootBall. */
  public ShootBall(double angle, boolean fullPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnBallHandler().getInstance());
    targetAngle = angle;
    this.fullPower = fullPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.returnBallHandler().setHoodBrake(false);
    RobotContainer.returnBallHandler().setFullPower(fullPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = targetAngle - RobotContainer.returnBallHandler().getHoodPosition();
    if (error > 0.8) {
      RobotContainer.returnBallHandler().setHoodBrake(false);
      RobotContainer.returnBallHandler().setHoodPower(-0.1);
      RobotContainer.returnBallHandler().spinFeeder(0);
    } else if (error < -0.8){
      RobotContainer.returnBallHandler().setHoodBrake(false);
      RobotContainer.returnBallHandler().setHoodPower(0.1);
      RobotContainer.returnBallHandler().spinFeeder(0);
    } else {
      RobotContainer.returnBallHandler().setHoodBrake(true);
      RobotContainer.returnBallHandler().setHoodPower(0);
      if (!fullPower) {
        if (Math.abs(24.5 - RobotContainer.returnBallHandler().getFlyWheelSpeed()) < 1.5) {
          RobotContainer.returnBallHandler().spinFeeder(0.3);
        } else {
          RobotContainer.returnBallHandler().spinFeeder(0);
        }
      } else {
        RobotContainer.returnBallHandler().setFlywheelPower();
        if (RobotContainer.returnRightJoy().getRawButton(10)) {
          RobotContainer.returnBallHandler().spinFeeder(0.3);
        } else {
          RobotContainer.returnBallHandler().spinFeeder(0);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnBallHandler().setHoodPower(0);
    RobotContainer.returnBallHandler().spinFeeder(0);
    RobotContainer.returnBallHandler().setHoodBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
