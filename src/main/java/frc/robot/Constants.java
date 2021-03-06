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
    // Motor IDs
    public static int motor_right1 = 0;
    public static int motor_right2 = 1;
    public static int motor_left1 = 3;
    public static int motor_left2 = 2;
    public static int motor_arm = 5;
    public static int motor_intake = 4;
    public static int motor_climbLeft = 6;
    public static int motor_climbRight = 7;

    // Encoder ratios
    public static double driveGearing = 1.0/60.0;
    public static double armGearing = 1.0/80.0;

    // Joystick Bindings
    public static int raiseButton = 6; // Right Bumper
    public static int lowerButton = 5; // Left Bumper
    public static int halfLowerButton = 4; // Y Button

    //Physical Constants (meters)
    public static double limelightHeight = 0.47;
    public static double hubHeight = 2.034; // 8 feet 8 inches is 104 inches, 71 for now bc the pit, * 0.0254 for m
    public static double limelightMountAngle = 0;
    public static double turnRadius = 0.2794;

    //Led Stuff
    public static int ledPort = 9;
}