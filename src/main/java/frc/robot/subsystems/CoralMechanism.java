
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralMechanism extends SubsystemBase {

    
    private final SparkMax coralMotor;
    /**
     * This subsytem that controls the arm.
     */
    public CoralMechanism(int CoralID) {
        coralMotor = new SparkMax(CoralID, MotorType.kBrushless);
    // Set up the arm motor as a brushless motor
    


    // Create and apply configuration for arm motor. Voltage compensation helps
    // the arm behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the arm stalls.
    // SparkMaxConfig coralConfig = new SparkMaxConfig();
    // coralConfig.voltageCompensation(10);
    // coralConfig.smartCurrentLimit(60);
    // coralConfig.idleMode(IdleMode.kBrake);
    // coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runIntake(double speed){
        coralMotor.set(speed);
    }
}
