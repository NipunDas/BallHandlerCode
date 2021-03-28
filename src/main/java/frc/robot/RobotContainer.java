// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveTrain m_drive = new DriveTrain();
  private static final BallHandler m_ballHandler = new BallHandler();

  private Command m_autoCommand;

  //joysticks and joystick buttons
  public static Joystick leftJoy = new Joystick(Constants.leftJoy);
  public static Joystick rightJoy = new Joystick(Constants.rightJoy);
  public static JoystickButton hood1, hood2, hood3, hood4;

  private static Limelight limelight = new Limelight();
  private static Ultrasonic ultrasonic = new Ultrasonic();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    hood1 = new JoystickButton(leftJoy, Constants.angle1Button);
    hood2 = new JoystickButton(leftJoy, Constants.angle2Button);
    hood3 = new JoystickButton(leftJoy, Constants.angle2Button);
    hood4 = new JoystickButton(leftJoy, Constants.angle2Button);
    hood1.whenPressed(new SetHood(BallHandler.hoodAngles[0]));
    hood2.whenPressed(new SetHood(BallHandler.hoodAngles[1]));
    hood3.whenPressed(new SetHood(BallHandler.hoodAngles[2]));
    hood4.whenPressed(new SetHood(BallHandler.hoodAngles[3]));
  }

  public static Joystick returnLeftJoy() {
    return leftJoy;
  }

  public static Joystick returnRightJoy() {
    return rightJoy;
  }

  public static Limelight getLimelight() {
    return limelight;
  }

  public static Ultrasonic getUltrasonic() {
    return ultrasonic;
  }

  public static DriveTrain returnDrive() {
    return m_drive;
  }

  public static BallHandler returnBallHandler() {
    return m_ballHandler;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if (limelight.getDistance() < 100) {
      m_autoCommand = new GalacticSearchRed();
    } else {
      m_autoCommand = new GalacticSearchBlue();
    }
    return m_autoCommand;
  }
}
