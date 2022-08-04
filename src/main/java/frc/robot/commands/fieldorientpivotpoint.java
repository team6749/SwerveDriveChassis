package frc.robot.commands;
import java.lang.annotation.Inherited;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.SwerveDrivebase;

public class fieldorientpivotpoint extends CommandBase {
    private final Translation2d point;
    private final SwerveDrivebase base;
    public fieldorientpivotpoint(Translation2d point, SwerveDrivebase base)
    {
        this.point = point;
        this.base = base;
        
    }
    @Override
    public void initialize()
    {
        // base.turning_point = point.minus(base.getPose2d().getTranslation());
        base.turning_point = (base.getPose2d().getTranslation());
        System.out.println((base.getPose2d().getTranslation()));
    }
    @Override
    public void execute()
    {
        //empty
    }
    public void end(boolean interrupted)
    {

    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
