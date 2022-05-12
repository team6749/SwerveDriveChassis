package frc.robot.subsystems;

public class SwerveDrivebase {
    //priv variables
    private SwerveModule frModule;
    private SwerveModule flModule;
    private SwerveModule brModule;
    private SwerveModule blModule;

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
    }
}
