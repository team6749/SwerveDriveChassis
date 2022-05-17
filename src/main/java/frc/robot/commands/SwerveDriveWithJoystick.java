// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


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
  private Joystick _right;
  private Joystick _left;

  /**
   * Creates a new ExampleCommand.
   * @param subsystem The subsystem used by this command.
   * @param right The right joystick
   * @param left The left Joystick
   */
  public SwerveDriveWithJoystick(SwerveDrivebase subsystem, Joystick left, Joystick right,) {
    _swerveDrivebase = subsystem;
    _right = right;
    _left = left;
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
    double xSpeed = _left.getX();
    double ySpeed = _left.getY();
    double directionalSpeed = _right.

    //apply the slew rate limiter

    //Construct the chassis speeds

    //convert the chassis speeds to individual module states

    //output each speed to the wheels
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
