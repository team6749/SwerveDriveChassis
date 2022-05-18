// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.subsystems.SwerveModule;

/** An example command that uses an example subsystem. */
public class SwerveDriveWithJoystick extends CommandBase {
  //private variables
  private SwerveDrivebase _swerveDrivebase;
  private SwerveModule swerveModule;
  private Joystick joystick;
  private SlewRateLimiter slewRateX = new SlewRateLimiter(5);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(5);

  /**
   * Creates a new ExampleCommand.
   * @param subsystem The subsystem used by this command.
   * @param joy The joystick used for controlling the swerve system
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
    double xSpeed = joystick.getX();
    double ySpeed = joystick.getY();
    double directionalSpeed = joystick.getTwist();

    //apply the slew rate limiter
    double limitSpeedX = slewRateX.calculate(xSpeed);
    double limitSpeedY = slewRateY.calculate(ySpeed);

    //Construct the chassis speeds
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(limitSpeedX, limitSpeedY, directionalSpeed);

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
