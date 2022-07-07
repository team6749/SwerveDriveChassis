// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivebase;

public class ResetGyro extends CommandBase {
  /** Creates a new ResetGyro. */
  private final SwerveDrivebase swerveDrivebase;
  public ResetGyro(SwerveDrivebase swerveDrivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivebase = swerveDrivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    swerveDrivebase.odometry.resetPosition(swerveDrivebase.odometry.getPoseMeters(), new Rotation2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}