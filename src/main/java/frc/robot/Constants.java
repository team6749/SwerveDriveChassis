// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    //swerve modules
    //[0] = speed/power motor;
    //[1] = angle/direction motor

    public static int[] frontRightModule = {};
    public static int[] frontLeftModule = {};
    public static int[] backRightModule = {};
    public static int[] backLeftModule = {};
    // locations for swerve modules relative to robot center.
    //measure from center of robot to center of modules
    public static Translation2d frontRightLocation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    public static Translation2d frontLeftLocation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    public static Translation2d backRightLocation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    public static Translation2d backLeftLocation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    //ports of the encoders
    public static int frontRightEncoder = 0;
    public static int frontLeftEncoder = 0;
    public static int backRightEncoder = 0;
    public static int backLeftEncoder = 0;
}
