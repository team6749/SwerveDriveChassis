// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ports relating to drive control
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    // 2024 falcon 500s ports - will need to change
    public static class ElectronicsPorts {
        public static final int frontLeftDrive = 1;  // TODO
        public static final int frontLeftEncoder = 2;  // TODO
        public static final int frontLeftAngle = 3;  // TODO
        public static final int frontRightDrive = 4;  // TODO
        public static final int frontRightEncoder = 5;  // TODO
        public static final int frontRightAngle = 6;  // TODO
        public static final int backRightDrive = 7;  // TODO
        public static final int backRightEncoder = 8;  // TODO
        public static final int backRightAngle = 9;  // TODO
        public static final int backLeftDrive = 10;  // TODO
        public static final int backLeftEncoder = 11;  // TODO
        public static final int backLeftAngle = 12;  // TODO
    }

    public static class SwerveConstants {
        // 2024 chassis constants, will need to change!
        public static double distFromCenterXMeters = .2525d; // TODO
        public static double distFromCenterYMeters = .2525d; // TODO
       
        // these two values should stay the same
        public static double swerveWheelDiameterMeters = .0978d;
        public static double swerveGearRatio = 8.14d;

        // precaution for rotation speed
        public static double turnMotorMaxOutputVolts = 7;

        // creating 4 modules to construct a swervedrivechassis with
        public static SwerveModule flModule = new SwerveModule(
                "Front Left",
                Constants.ElectronicsPorts.frontLeftDrive,
                Constants.ElectronicsPorts.frontLeftEncoder,
                Constants.ElectronicsPorts.frontLeftAngle,
                new Translation2d(distFromCenterXMeters, distFromCenterYMeters));

        public static SwerveModule frModule = new SwerveModule(
                "Front Right",
                Constants.ElectronicsPorts.frontRightDrive,
                Constants.ElectronicsPorts.frontRightEncoder,
                Constants.ElectronicsPorts.frontRightAngle,
                new Translation2d(distFromCenterXMeters, -distFromCenterYMeters));
        public static SwerveModule brModule = new SwerveModule(
                "Back Right",
                Constants.ElectronicsPorts.backRightDrive,
                Constants.ElectronicsPorts.backRightEncoder,
                Constants.ElectronicsPorts.backRightAngle,
                new Translation2d(-distFromCenterXMeters, -distFromCenterYMeters));
        public static SwerveModule blModule = new SwerveModule(
                "Back Left",
                Constants.ElectronicsPorts.backLeftDrive,
                Constants.ElectronicsPorts.backLeftEncoder,
                Constants.ElectronicsPorts.backLeftAngle,
                new Translation2d(-distFromCenterXMeters, distFromCenterYMeters));
        public static SwerveModule[] swerveModuleArray = { flModule, frModule, brModule, blModule };

    }

    // copied from 2024 code
    public static class JoystickConstants {
        public static final double deadZoneRange = 0.15;
        public static final double deadZoneRotation = 0.10;

        public static final double maxLinearSpeedms = 4.0;
        public static final double maxRotationalSpeedDegrees = 360;

        public static final double joystickLinearityAdjustment = 0.8;
    }

    public static double kMaxRobotVelocity = 2.75;
}
