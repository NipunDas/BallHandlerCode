// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //talons
    public static int leftDrivePort = 2;
    public static int rightDrivePort = 3;
    public static int leftSlavePort = 4;
    public static int rightSlavePort = 5;
    public static int intakePort = 10;
    public static int feederPort = 7;
    public static int flywheelPort = 14;
    public static int hoodPort = 9;
    
    //joystick and joystick buttons
    public static int leftJoy = 0;
    public static int rightJoy = 1;
    public static int angle1Button = 8;
    public static int angle2Button = 9;
    public static int angle3Button = 10;
    public static int intakeToggle = 6;
    public static int alignButton = 1;
}
