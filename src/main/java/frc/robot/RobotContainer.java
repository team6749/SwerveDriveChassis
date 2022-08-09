// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SwerveDriveWithJoystick;
import frc.robot.commands.setpivotpoint;
import frc.robot.subsystems.SwerveDrivebase;

import java.util.List;
import java.util.Random;
import java.lang.Math;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.*;
import frc.robot.subsystems.SwerveModule;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  


  public final static SwerveDrivebase swerveDrivebase = new SwerveDrivebase(new SwerveModule[] {Constants.flModule, Constants.blModule, Constants.frModule, Constants.brModule});
  public static Joystick _joystick = new Joystick(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    swerveDrivebase.setDefaultCommand(new SwerveDriveWithJoystick(swerveDrivebase, _joystick));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton resetGyro = new JoystickButton(_joystick, 1);
    resetGyro.whenPressed(new ResetGyro(swerveDrivebase));
    final JoystickButton frontleftpivot = new JoystickButton(_joystick, 5);
    frontleftpivot.whenPressed(new setpivotpoint(Constants.flModule.position, swerveDrivebase));
   
    final JoystickButton backleftpivot = new JoystickButton(_joystick, 3);
    backleftpivot.whenPressed(new setpivotpoint(Constants.blModule.position, swerveDrivebase));
    final JoystickButton backrightpivot = new JoystickButton(_joystick, 4);
    backrightpivot.whenPressed(new setpivotpoint(Constants.brModule.position, swerveDrivebase));
    final JoystickButton frontrightpivot = new JoystickButton(_joystick, 6);
    frontrightpivot.whenPressed(new setpivotpoint(Constants.frModule.position, swerveDrivebase));
    final JoystickButton centerpivot = new JoystickButton(_joystick, 7);//button 6 is labelled as "7" on the joystick
    centerpivot.whenPressed(new setpivotpoint(new Translation2d(0,0), swerveDrivebase));

  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(5 / 4, 3).setKinematics(swerveDrivebase._kinematics);
    int rand = (int)(Math.random() * (360) + 1);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      //negative translation2d is right of opening
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(1, 0),
              new Translation2d(1, -1),
              new Translation2d(0, -1)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(rand)),
      trajectoryConfig);

      PIDController xController = new PIDController(6.5, 0, 0);
      PIDController yController = new PIDController(6.5, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              3, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveDrivebase::getPose2d,
        swerveDrivebase._kinematics,
        xController,
        yController,
        thetaController,
        swerveDrivebase::setModuleStates,
        swerveDrivebase);

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveDrivebase.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveDrivebase.stopModules()));
  }

}
