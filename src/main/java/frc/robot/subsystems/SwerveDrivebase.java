package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivebase extends SubsystemBase{
    //priv variables
    public SwerveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    // constructor
    /**
     * @param mods - An Array of SwerveDriveModules
     * Creates a new SwerveDrivebase
     */
    public SwerveDrivebase(SwerveModule[] mods){
        modules = mods;
        //constructs a drivebase using translation2ds assigned to each module
        Translation2d[] translations = new Translation2d[mods.length];
        for(int i = 0; i < translations.length; i++){
            translations[i] = mods[i].position;
        }
        _kinematics = new SwerveDriveKinematics(translations);
    }

    @Override
    public void periodic() {
        //Call periodic on children
        for (SwerveModule swerveModule : modules) {
            swerveModule.periodic();
        }
    }

    public void setDesiredChassisSpeeds(ChassisSpeeds cSpeeds) {
        SwerveModuleState[] _desiredStates = _kinematics.toSwerveModuleStates(cSpeeds);
        for (int i = 0; i < _desiredStates.length; i++) {
            modules[i].setDesiredState(_desiredStates[i]);
        }
    }
}
