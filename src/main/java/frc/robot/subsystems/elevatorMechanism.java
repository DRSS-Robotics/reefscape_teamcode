package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class ElevatorMechanism extends SubsystemBase {

    ClosedLoopConfig ElevCCLoop = new ClosedLoopConfig().p(0.15).i(0.0).d(0.0);
    SparkMaxConfig ElevConfig = new SparkMaxConfig();

    public final double Deadband = 0.04;
    public final SparkMax elevatorMotor;
    boolean IsAtTarget = false;
    boolean ShouldGoUp = false;
    public int DesiredCoralHeight;
    // This first one is the coral station and the last one is level 3
    // Change the coral station level
    public double[] CoralHeights = { 20, 15.0, 110.0 }; 
    
    boolean PrevElevDir = true;
    /**
     * This subsytem that controls the arm.
     */
    public ElevatorMechanism() {
        
        elevatorMotor = new SparkMax(13, MotorType.kBrushless);
    // Set up the arm motor as a brushed motor
    


    // Create and apply configuration for arm motor. Voltage compensation helps
    // the arm behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the arm stalls.

    ElevConfig.voltageCompensation(10);
    ElevConfig.smartCurrentLimit(60);
    ElevConfig.idleMode(IdleMode.kBrake);
    elevatorMotor.configure(ElevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        DriveElevator(RobotContainer.Controller2);
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
        elevatorMotor.getEncoder().getPosition() * Constants.kElevatorDrumRadius * 2 * Math.PI;
 
        // also in meters - what else did you think it would be in?
        double MotorVelocityLinear =
        elevatorMotor.getEncoder().getVelocity() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        // System.out.println(MotorPositionLinear);
        // if (MotorPositionLinear + MotorVelocityLinear >= Constants.kMaxElevatorHeightMeters ||
        //     MotorPositionLinear + MotorVelocityLinear < Constants.kMinElevatorHeightMeters) {

        //Change values of this later for deep climb
        if (elevatorMotor.getEncoder().getPosition() - 8 * ControllerY < -3 ||
            elevatorMotor.getEncoder().getPosition() - 8 * ControllerY >= -120) {
            return false;
        } else {
            return true;
        }
    }


        public Command DriveElevator(CommandXboxController Controller) {
        if (DeadbandCheck(Controller.getLeftY()) && MotorHeightBounds(Controller.getRightY())) {
            elevatorMotor.set(-Controller.getRightY());
            return Commands.run(() -> elevatorMotor.set(-Controller.getRightY()));
        } else {
            elevatorMotor.stopMotor();
            return Commands.run(() -> elevatorMotor.stopMotor());
        }
    }

    public Command SetDesiredElevatorHeight(int CoralLevelIndex) {
        IsAtTarget = false;
        DesiredCoralHeight = CoralLevelIndex;
        System.out.println("Guhh");
        return Commands.none();
    }
        public Command DriveToPosition(double InPos) {
        // ShouldGoUp = (Elevator.getEncoder().getPosition() <
        // CoralHeights[DesiredCoralHeight]);
        elevatorMotor.getClosedLoopController().setReference(InPos, ControlType.kPosition);
        return Commands.none();
    }


}

