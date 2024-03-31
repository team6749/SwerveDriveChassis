// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.DriveOrientation;

@SuppressWarnings("unused")
public class SwerveDrivebase extends SubsystemBase {
    /** Creates a new SwerveDrivebase. */
    public SwerveModule[] modules;
    public SwerveDriveKinematics kinematics;
    public SwerveModuleState[] states;
    public SwerveDrivePoseEstimator poseEstimator;
    public DriveOrientation selectedOrientation;

    public ADIS16470_IMU gyro = new ADIS16470_IMU(); //2024 gyro

    public final Field2d field = new Field2d();

    boolean useLimelightDuringAutos = true; // option on shuffleboard to use limelight during autonomous mode. If autos are off or going to the wrong spots on occasions, you may want to turn this to false

    NetworkTable limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight"); // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

    /**
     * constructs a new swerve drivebase comprised of 2 or more modules (typically
     * four)
     * 
     * @param modules an array of modules in a drivebase.
     */
    public SwerveDrivebase(SwerveModule[] modules) {
        this.modules = modules;

        gyro.calibrate();

        Translation2d[] translations = new Translation2d[modules.length];

        for (int i = 0; i < translations.length; i++) {
            translations[i] = modules[i].location;
        }

        for (SwerveModule swerveModule : modules) {
            SmartDashboard.putData(swerveModule.name, swerveModule);
        }

        kinematics = new SwerveDriveKinematics(translations);

        // measurement from just the wheels

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), getCurrentModulePositions(),
                new Pose2d(0, 0, getRotation2d()));

        selectedOrientation = DriveOrientation.RobotOriented;

        SmartDashboard.putData("field map", field);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            // Call periodic on all modules
            module.periodic();
        }

        // This method will be called once per scheduler run
        poseEstimator.update(getRotation2d(), getCurrentModulePositions());

        // Attempt to set bot pose using limelight vision measurement
        try {
            NetworkTableEntry botPose = limelightNetworkTable.getEntry("botpose_wpiblue");
            double[] botPoseArray = botPose.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0 }); // Translation(x,y,z),
                                                                                                  // Rotation(roll,
                                                                                                  // pitch, yaw),
                                                                                                  // full latency
            Pose2d estimatedPosition = new Pose2d(botPoseArray[0], botPoseArray[1],
                    Rotation2d.fromDegrees(botPoseArray[5]));
            double currentTime = Timer.getFPGATimestamp() - (botPoseArray[6] / 1000.0); // latency

            ChassisSpeeds speeds = getSubsystemChassisSpeeds();
            Translation2d zoom = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); // current chassis speeds
            Rotation2d zoomrot = new Rotation2d(speeds.omegaRadiansPerSecond); // current rotationals speeds
            if ((RobotState.isAutonomous() && getUseLimelightDuringAutos()) || RobotState.isTeleop()
                    || RobotState.isTest()) {
                // this  determines how much we should trust the limelight
                // for example, it could be less accurate due to moving fast past an april tag and thinking its next to that april
                // tag after it has passed it
                if (botPoseArray[0] != 0 && zoom.getNorm() < 1.5 && Math.abs(zoomrot.getDegrees()) < 20) {
                    // trust vision less, maybe
                    poseEstimator.setVisionMeasurementStdDevs(MatBuilder.fill(Nat.N3(), Nat.N1(),
                            8, 8, 16)); // this 8,8,16 is a measurement of how little or how much we should trust the april tag measurments
                    poseEstimator.addVisionMeasurement(estimatedPosition, currentTime);
                }
            }
        } catch (Exception e) {
            System.out.println("THE LIMELIGHT CODE CRASHED");
        }

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    /**
     * adds all values in the function to shuffleboard under the domain
     * "SwerveSubsystem"
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveSubsystem");
        SmartDashboard.putData("gyro", gyro);
        builder.addStringProperty("Orientation", () -> getSelectedDriveMode().toString(), null);
        builder.addBooleanProperty("Use limelight during autos?", this::getUseLimelightDuringAutos,
                this::setUseLimelightDuringAutos);
    }

    /**
     * 
     * Converts chassisSpeeds for each module into SwerveModuleState. Calls
     * setSubsystemModuleStates with chassisSpeeds converted to SwerveModuleStates
     * 
     * @param chassisSpeeds the desired chassis speeds to set each module to
     */
    public void setSubsystemChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setSubsystemModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * sets each module to the desired SwerveModuleState
     * 
     * @param desiredStates an array of desired module SwerveModuleStates.
     */
    public void setSubsystemModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < desiredStates.length; i++) {
            modules[i].setSwerveModuleState(desiredStates[i]);
        }
    }

    /**
     * 
     * @return an array of current module positions - used for updating the robot
     *         position on the field
     */
    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getModulePosition();
        }
        return modulePositions;
    }

    /**
     * 
     * @return a Pose2d ( x, y ) of the robot position IN METERS
     */
    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * 
     * @return a pose3d of the robots position on the field, and its rotation
     */
    public Pose3d getPose3d() {
        return new Pose3d(new Translation3d(getPose2d().getX(), getPose2d().getY(), 0),
                new Rotation3d(0, 0, getRotation2d().getRadians()));
    }

    /**
     * 
     * @return a Rotation2d of the robots current rotation
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)); // TODO Check this
    }


    /**
     * converts the SwerveModuleStates of each module to
     * 
     * @return the current ChassisSpeeds of each module
     */
    public ChassisSpeeds getSubsystemChassisSpeeds() {
        SwerveModuleState[] currentModuleStates = new SwerveModuleState[this.modules.length];
        // gets current chassis state for each module
        for (int i = 0; i < this.modules.length; i++) {
            currentModuleStates[i] = modules[i].getModuleState();
        }
        return kinematics.toChassisSpeeds(currentModuleStates);
    }

    /**
     * resets the Position on the field of the Robot
     * used for the robot autonomous in order to reset the saved position on field
     * 
     * @param pose2d current position of the robot
     */
    public void resetOdometry(Pose2d pose2d) {
        System.out.println("Resetting Odometry at " + pose2d.getX() + " " + pose2d.getY());

        poseEstimator.resetPosition(
                getRotation2d(),
                getCurrentModulePositions(),
                pose2d);
        System.out.println(poseEstimator.getEstimatedPosition().getX() + " "
                + poseEstimator.getEstimatedPosition().getY() + " " + poseEstimator.getEstimatedPosition().toString());
    }

    /**
     * loops through each module and running stop(), a function to cut power to the
     * motors
     */
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    /**
     * gets the driveMode orientation
     * 
     * @return the current select DriveMode, either RobotOriented or FieldOriented
     */
    public DriveOrientation getSelectedDriveMode() {
        return selectedOrientation;
    }

    /**
     * sets the driveMode orientation
     */
    public void setSelectedDriveMode(DriveOrientation newOrientation) {
        selectedOrientation = newOrientation;
    }

    private void toggleSelectedDriveMode() {
        if (selectedOrientation == DriveOrientation.FieldOriented) {
            setSelectedDriveMode(DriveOrientation.RobotOriented);
        } else {
            setSelectedDriveMode(DriveOrientation.FieldOriented);
        }
    }

    public Command driveModeCommand() {
        return runOnce(() -> toggleSelectedDriveMode());
    }

    public Command resetOdometryCommand() {
        return runOnce(() -> resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))));
    }

    /**
     * sets each module to the NeutralModeValue - either Break or coast
     * Coast moves without locking the wheels, Break locks the wheels and resists
     * friction
     * 
     * @param neutralModeValue the NeutralMode to set the robot to
     */
    public void setModulesNeutralMode(NeutralModeValue neutralModeValue) {
        for (int i = 0; i < this.modules.length; i++) {
            modules[i].setModuleNeutralMode(neutralModeValue);
        }
    }


    /**
     * 
     * @return the current value of the boolean whether to use limelight in autos
     */
    public boolean getUseLimelightDuringAutos() {
        return useLimelightDuringAutos;
    }

    /**
     * resets the value of useLimelightDuringAutos
     * 
     * @param useLimelightDuringAutos the new value to set auto usage state
     */
    public void setUseLimelightDuringAutos(boolean useLimelightDuringAutos) {
        this.useLimelightDuringAutos = useLimelightDuringAutos;
    }
}