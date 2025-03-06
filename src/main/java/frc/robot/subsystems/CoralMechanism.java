package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralMechanism extends SubsystemBase {

    public SparkMax Elevator;
    public SparkMax Intake;

    public boolean ElevatorActivated = false;
    public final double Deadband = 0.04;
    
    // placeholders 'til we actually measure & calulate, not actually implemented right now
    public int MinElevEncoderHeight = 0;
    public int MaxElevEncoderHeight = 3;
    SparkMax Elevator;
    SparkMax Intake;

    public CoralMechanism(int ElevID, int IntakeID) {
        // Elevator = new SparkMax(ElevID, MotorType.kBrushless);
        // Intake = new SparkMax(IntakeID, MotorType.kBrushless);
    }

    public boolean DeadbandCheck(double Value) {
        return Math.abs(Value) > Deadband;
    }

    public Command SetIntakeSpeed(double Speed) {
        return Commands.runOnce(() -> Intake.set(Speed));
    }

    public Command DriveElevator(double Speed) {
        if (DeadbandCheck(Speed)) {
            return Commands.runOnce(() -> /*Elevator.set(-0.5 * Speed)*/ System.out.println(Speed));
        } else {
            return Commands.runOnce(() -> /*Elevator.set(0)*/ System.out.println("stop"));
        }
    }

    public Command DriveElevator(double Speed) {
        if (DeadbandCheck(Speed)) {
            return Commands.runOnce(() -> /*Elevator.set(-0.5 * Speed)*/ System.out.println(Speed));
        } else {
            return Commands.runOnce(() -> /*Elevator.set(0)*/ System.out.println("stop"));
        }
    }

    public Command DriveElevator(CommandXboxController Controller) {
        if (DeadbandCheck(Controller.getLeftY())) {
            return Commands.runOnce(() -> /*Elevator.set(-0.5 * Controller.getLeftY())*/ System.out.println(Controller.getLeftY()));
        } else {
            return Commands.runOnce(() -> /*Elevator.set(0)*/ System.out.println("stop"));
        }
    }
   }