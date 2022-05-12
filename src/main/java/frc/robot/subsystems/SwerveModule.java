package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

public class SwerveModule {
    // private variables 
    private final WPI_TalonFX speedMotor; 
    private final WPI_TalonFX angleMotor;
    private final AnalogInput absoluteEncoder;

    //constructor
    /**
     * @param speedMotor - the port of the speed motor
     * @param angleMotor - the port of the angle motor
     * Creates a new SwerveModule
     */
    public SwerveModule(int speedPort, int anglePort, int absEncoderPort){
        this.speedMotor = new WPI_TalonFX(speedPort);
        this.angleMotor = new WPI_TalonFX(anglePort);
        this.absoluteEncoder = new AnalogInput(absEncoderPort);
    }
}
