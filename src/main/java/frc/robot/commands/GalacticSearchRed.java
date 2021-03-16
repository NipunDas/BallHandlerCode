// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchRed extends SequentialCommandGroup {
  /** Creates a new GalacticSearchRed. */
  public GalacticSearchRed() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForward(1),
                new AlignBall(),
                new DriveForward(1),
                new FindPowerCell(true),
                new DriveForward(0.5),
                new AlignBall(),
                new DriveForward(1.75),
                new FindPowerCell(false),
                new DriveForward(1.5),
                new AlignBall(),
                new DriveForward(1),
                new TurnRobot(72),
                new DriveForward(3.9));
  }
}
