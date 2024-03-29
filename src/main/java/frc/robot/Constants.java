// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModule;

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
    // absolute encoder port
    // calibration of angle sensor
    // pos = translation2d distance from the center
    public static SwerveModule flModule = new SwerveModule(
        "Front Left",
        5, 
        6, 
        10, 
        105.985, 
        new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(11.25))
    );
    public static SwerveModule blModule = new SwerveModule(
        "Back Left", 
        7, 
        8, 
        11, 
        303.43, 
        new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(11.25))
    );
    public static SwerveModule frModule = new SwerveModule(
        "Front Right", 
        3, 
        4, 
        12, 
        319.7363, 
        new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(-11.25))
    );
    public static SwerveModule brModule = new SwerveModule(
        "Back Right", 
        1, 
        2, 
        9, 
        292.913, 
        new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(-11.25))
    );
    
    public static double kMaxRobotVelocity = 2.75;
}
