package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class hangMechanism extends SubsystemBase {

    public final double Deadband = 0.04;
    private final SparkMax hangMechanism;
    /**
     * This subsytem that controls the arm.
     */
    public hangMechanism() {
        
        hangMechanism = new SparkMax(12, MotorType.kBrushless);
    // Set up the arm motor as a brushed motor
    


    // Create and apply configuration for arm motor. Voltage compensation helps
    // the arm behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the arm stalls.
    SparkMaxConfig hangConfig = new SparkMaxConfig();
    hangConfig.voltageCompensation(10);
    hangConfig.smartCurrentLimit(60);
    hangConfig.idleMode(IdleMode.kBrake);
    hangMechanism.configure(hangConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        DriveHang(RobotContainer.Controller2);
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */


    public boolean DeadbandCheck(double Value) {
        //System.out.println(Math.abs(Value) > Deadband);
        return Math.abs(Value) > Deadband;
    }


    
        public boolean MotorHeightBounds(double ControllerY) {
        // in meters
        double MotorPositionLinear =
        hangMechanism.getEncoder().getPosition() * Constants.kElevatorDrumRadius * 2 * Math.PI;
 
        // also in meters - what else did you think it would be in?
        double MotorVelocityLinear =
        hangMechanism.getEncoder().getVelocity() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        // System.out.println(MotorPositionLinear);
        // if (MotorPositionLinear + MotorVelocityLinear >= Constants.kMaxElevatorHeightMeters ||
        //     MotorPositionLinear + MotorVelocityLinear < Constants.kMinElevatorHeightMeters) {

        //Change values of this later for deep climb
        if (hangMechanism.getEncoder().getPosition() - 8 * ControllerY < -3 ||
            hangMechanism.getEncoder().getPosition() - 8 * ControllerY >= -120) {
            return false;
        } else {
            return true;
        }
    }


        public Command DriveHang(CommandXboxController Controller) {
        if (DeadbandCheck(Controller.getLeftY()) && MotorHeightBounds(Controller.getRightY())) {
            hangMechanism.set(-Controller.getRightY());
            return Commands.run(() -> hangMechanism.set(-Controller.getRightY()));
        } else {
            hangMechanism.stopMotor();
            return Commands.run(() -> hangMechanism.stopMotor());
        }
    }


}
