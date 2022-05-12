package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class SwerveModule {
    // private variables 
    private final WPI_TalonFX speedMotor; 
    private final WPI_TalonFX angleMotor; 

    //constructor
    /**
     * @param speedMotor - the port of the speed motor
     * @param angleMotor - the port of the angle motor
     * Creates a new SwerveModule
     */
    public SwerveModule(int speedPort, int anglePort){
        this.speedMotor = speedPort;
        this.angleMotor = anglePort;
    }
}
