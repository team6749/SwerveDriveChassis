package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveDrivebase {
    //priv variables
    public SwerveModule frModule;
    public SwerveModule flModule;
    public SwerveModule brModule;
    public SwerveModule blModule;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    // constructor
    /**
     * @param fr - the front right module
     * @param fl - the front left module
     * @param br - the back right module
     * @param bl - the back left module
     * Creates a new SwerveDrivebase
     */
    public SwerveDrivebase(SwerveModule fr, SwerveModule fl, SwerveModule br, SwerveModule bl){
        this.frModule = fr;
        this.flModule = fl;
        this.brModule = br;
        this.blModule = bl;
        this._kinematics = new SwerveDriveKinematics(
            Constants.frontRightLocation,
            Constants.frontLeftLocation,
            Constants.backRightLocation, 
            Constants.backLeftLocation
        );
        //converting the locations of the SwerveModuleStates
        //[0] = fr; [1] = fl; [2] = br; [3] = bl;
        this.states = this._kinematics.toSwerveModuleStates(chassisSpeeds);
    }
}
