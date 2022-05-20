package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrivebase extends SubsystemBase{
    //priv variables
    public SwerveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    // constructor
    /**
     * @param modules - An Array of SwerveDriveModules
     * Creates a new SwerveDrivebase
     */
    public SwerveDrivebase(SwerveModule[] modules){
        this.modules = modules;
        //constructs a drivebase using translation2ds assigned to each module
        Translation2d[] translations = new Translation2d[modules.length];
        for(int i = 0; i < translations.length; i++){
            translations[i] = modules[i].position;
        }
        _kinematics = new SwerveDriveKinematics(translations);
    }

    public Rotation2d getChassisRotation(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    @Override
    public void periodic() {
        //Call periodic on children
        for (SwerveModule swerveModule : modules) {
            swerveModule.periodic();
        }
    }
    /**
     * @param cSpeeds the desired chassis speeds to set each module to
     * converts each module to the ChassisSpeeds cSpeeds
     */
    public void setDesiredChassisSpeeds(ChassisSpeeds cSpeeds) {
        SwerveModuleState[] _desiredStates = _kinematics.toSwerveModuleStates(cSpeeds);
        for (int i = 0; i < _desiredStates.length; i++) {
            modules[i].setDesiredState(_desiredStates[i]);
        }
    }
}
