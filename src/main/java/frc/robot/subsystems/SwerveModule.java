package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {
    // private variables 
    private final WPI_TalonFX speedMotor; 
    private final WPI_TalonFX angleMotor;
    private final Encoder absoluteEncoder;
    public final PIDController _PidController = new PIDController(0, 0, 0);

    //constructor
    /**
     * @param speedMotor - the port of the speed motor
     * @param angleMotor - the port of the angle motor
     * Creates a new SwerveModule
     */
    public SwerveModule(int speedMotorId, int angleMotorId, int absEncoderPort){
        this.speedMotor = new WPI_TalonFX(speedMotorId);
        this.angleMotor = new WPI_TalonFX(angleMotorId);
        this.absoluteEncoder = new Encoder(absEncoderPort);
        _PidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        //replace _driveEncoder with the speedMotor encoder
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(absoluteEncoder.get()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(absoluteEncoder.get()));
    
        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            _PidController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
    
        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            _PidController.calculate(m_turningEncoder.get(), state.angle.getRadians());
    
        final double turnFeedforward =
            m_turnFeedforward.calculate(_PidController.getSetpoint().velocity);
    
        speedMotor.set(driveOutput + driveFeedforward);
        angleMotor.set(turnOutput + turnFeedforward);
      }
}
