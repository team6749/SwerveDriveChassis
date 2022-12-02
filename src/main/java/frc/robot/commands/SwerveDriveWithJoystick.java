// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private SlewRateLimiter slewRateX = new SlewRateLimiter(3.85); 
  private SlewRateLimiter slewRateY = new SlewRateLimiter(4.75); 
  public SendableChooser<String> orientation = new SendableChooser<String>(); 
  public ChassisSpeeds desiredSpeeds;

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

    orientation.setDefaultOption("Robot Oriented", "RO");
    orientation.addOption("Field Oriented", "FO");
    SmartDashboard.putData(orientation);
    addRequirements(_swerveDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get the joystick output values
    double xSpeed = joystick.getY();//xonjoystick
    double ySpeed = joystick.getX()*.2;//Y ON JOYSTICK
    double directionalSpeed = joystick.getTwist() * 2.0 ;

    //applies a controller deadzone 
    if(new Translation2d(xSpeed, ySpeed).getNorm() < 0.1 && Math.abs(directionalSpeed) < 0.1) {
      _swerveDrivebase.setDesiredChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }

    //apply the slew rate limiter
    double limitSpeedX = slewRateX.calculate(xSpeed);
    double limitSpeedY = slewRateY.calculate(ySpeed);

    //Construct the chassis speeds
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, directionalSpeed);

    //output each speed to the wheels
    _swerveDrivebase.setDesiredChassisSpeeds(desiredSpeeds);
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
