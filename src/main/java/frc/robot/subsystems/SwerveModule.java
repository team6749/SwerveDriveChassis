package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // private variables 
    private final WPI_TalonFX driveMotor; 
    private final WPI_TalonFX angleMotor;
    private final WPI_CANCoder absoluteEncoder;
    public final PIDController _PidController = new PIDController(0, 0, 0);
    public Translation2d position;
    public double calibrationDegrees;
    public String name;
    //constructor
    /**
     * @param speedMotorId - the port of the speed motor
     * @param angleMotorId - the port of the angle motor
     * @param absEncoderPort - the port of the absolute encoder
     * @param calibrationDegrees - the offset in degrees of the abs encoder
     * @param pos - the Translation2d distance on x/y from the center of the robot
     * Creates a new SwerveModule
     */
    public SwerveModule(String name, int speedMotorId, int angleMotorId, int absEncoderPort, double calibrationDegrees, Translation2d pos){
        this.driveMotor = new WPI_TalonFX(speedMotorId);
        this.angleMotor = new WPI_TalonFX(angleMotorId);
        this.absoluteEncoder = new WPI_CANCoder(absEncoderPort);
        this.calibrationDegrees = calibrationDegrees;
        this.position = pos;
        this.name = name;
        _PidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    //Meters per second
    public double getDriveEncoderVelocity() {
        return 2048 * 1/8.14 * driveMotor.getSelectedSensorVelocity();
    }

    public double getDriveEncoderPos(){
        return 2048 * 1/8.14 * driveMotor.getSelectedSensorPosition();
    }

    //Degrees
    public double getRotationEncoder() {
        return absoluteEncoder.getAbsolutePosition() + calibrationDegrees;
    }

    //This must be called all the time
    public void periodic () {
        SmartDashboard.putNumber("Swerve " + name + " rotation", getRotationEncoder());
        SmartDashboard.putNumber("Swerve " + name + " velocity", getDriveEncoderVelocity());
    }


    public SwerveModuleState getState() {
        //replace _driveEncoder with the speedMotor encoder
        return new SwerveModuleState(getDriveEncoderVelocity(), new Rotation2d(getRotationEncoder()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getRotationEncoder()));

                    // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
        _PidController.calculate(getRotationEncoder(), state.angle.getDegrees());
    
        // // Calculate the drive output from the drive PID controller.
        // final double driveOutput =
        //     _PidController.calculate(getDriveEncoder(), state.speedMetersPerSecond);
    
        // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
        // final double turnFeedforward =
        //     m_turnFeedforward.calculate(_PidController.getSetpoint().velocity);
    
        // speedMotor.set(driveOutput + driveFeedforward);
        // angleMotor.set(turnOutput + turnFeedforward);

        angleMotor.set(turnOutput);
        driveMotor.set(state.speedMetersPerSecond * 0.5);
    }
}
