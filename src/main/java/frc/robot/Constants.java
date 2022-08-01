// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static int motor_right1 = 2;
    public static int motor_right2 = 3;
    public static int motor_left1 = 1;
    public static int motor_left2 = 4;
    public static int motor_arm = 8;
    public static int motor_intake = 5;
    public static int motor_climbLeft = 6;
    public static int motor_climbRight = 7;

    // Encoder ratios (both very very wrong)
    public static double driveGearing = 1.0/60.0;
    public static double armGearing = 1.0/80.0;
    
    //public static double driveGearing = 10.71/1;
    //public static double armGearing = 16/1; 
    //^^ie the motor needs to spin 16 times for the arm to turn once

    public static String circleTrajectoryJSON = "paths/Circle.wpilib.json";

    //currently unrealistic values because we screwed up testing
    public static final double ksVolts = 0.59348;
    public static final double kvVoltSecondsPerMeter = 2.1763; // 2.1763;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15926;
    public static final double kPDriveVel = 2.457;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    //^^these might have to be tuned

    //tutorial said these values were fine
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //distance between centers of the left side center of one wheel to the right center of the other wheel, across the robot chassis
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.875);
    public static DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    
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