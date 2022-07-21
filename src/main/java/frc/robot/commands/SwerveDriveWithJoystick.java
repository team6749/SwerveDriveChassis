// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivebase;

/** An example command that uses an example subsystem. */
public class SwerveDriveWithJoystick extends CommandBase {
  //private variables
  private SwerveDrivebase _swerveDrivebase;
  private Joystick joystick;
  private SlewRateLimiter slewRateX = new SlewRateLimiter(5);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(5);

  /**
   * Creates a new SwerveDriveWithJoystick command.
   * @param subsystem The subsystem used by this command.
   * @param joy The joystick used for controlling the swerve system
   * drives the swerve robot with only 1 joystick
   */
  public SwerveDriveWithJoystick(SwerveDrivebase subsystem, Joystick joy) {
    _swerveDrivebase = subsystem;
    joystick = joy;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_swerveDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get the joystick output values
    double xSpeed = -joystick.getY();
    double ySpeed = -joystick.getX();
    double directionalSpeed = -joystick.getTwist() * 2.5;

    // Dead zone to rotation input
    if(Math.abs(directionalSpeed) <= 0.5 ) {
      directionalSpeed = 0;
    }

    //applies a controller deadzone 
    if(new Translation2d(xSpeed, ySpeed).getNorm() < 0.1 && Math.abs(directionalSpeed) < 0.1) {
      _swerveDrivebase.setDesiredChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }

    //apply the slew rate limiter
    double limitSpeedX = slewRateX.calculate(xSpeed);
    double limitSpeedY = slewRateY.calculate(ySpeed);

    //Construct the chassis speeds - robot oriented
    // ChassisSpeeds desiredSpeeds = new ChassisSpeeds(limitSpeedX, limitSpeedY, directionalSpeed);

    //field oriented drive chassis speeds
    ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(limitSpeedX, limitSpeedY, directionalSpeed, _swerveDrivebase.getPose2d().getRotation());

    //if(joystick.getRawButton(button))
    System.out.println(_swerveDrivebase.turning_point.toString());
    //output each speed to the wheels
    //_swerveDrivebase.setDesiredChassisSpeeds(desiredSpeeds, _swerveDrivebase.getPose2d().getTranslation().rotateBy(_swerveDrivebase.getPose2d().getRotation()));
    _swerveDrivebase.setDesiredChassisSpeeds(desiredSpeeds, _swerveDrivebase.turning_point);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //never true since default command
    return false;
  }
}
