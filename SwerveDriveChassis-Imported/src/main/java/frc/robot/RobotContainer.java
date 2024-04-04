// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveWithController;
import frc.robot.subsystems.SwerveDrivebase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrivebase swerveDrivebase;
    private final XboxController controller;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        swerveDrivebase = new SwerveDrivebase(Constants.SwerveConstants.swerveModuleArray);
        controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

        SmartDashboard.putData("Swerve Subsystem", swerveDrivebase);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // sets the default command of swerve drivebase to sdwc
        // this command should never return true for isFinished
        swerveDrivebase.setDefaultCommand(new SwerveDriveWithController(swerveDrivebase, controller));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     */
    // public Command getAutonomousCommand() {
    // // An ExampleCommand will run in autonomous
    // }

}
