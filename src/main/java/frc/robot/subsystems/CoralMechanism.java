package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class CoralMechanism extends SubsystemBase {

    SparkMax Elevator;
    SparkMax Intake;

    public final double Deadband = 0.04;
    public CoralMechanism(int ElevID, int IntakeID) {
        Elevator = new SparkMax(ElevID, MotorType.kBrushless);
        Elevator.getEncoder().setPosition(0);
        Intake = new SparkMax(IntakeID, MotorType.kBrushless);
    }

    public boolean DeadbandCheck(double Value) {
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds() {
        // in meters
        double MotorPositionLinear =
        Elevator.getEncoder().getPosition() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        // also in meters - what else did you think it would be in?
        double MotorVelocityLinear =
        Elevator.getEncoder().getVelocity() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        System.out.println(MotorPositionLinear);
        if (MotorPositionLinear + MotorVelocityLinear >= Constants.kMaxElevatorHeightMeters ||
            MotorPositionLinear + MotorVelocityLinear < Constants.kMinElevatorHeightMeters) {
            Elevator.stopMotor();
            return false;
        } else {
            return true;
        }
    }

    public Command SetIntakeSpeed(double Speed) {
        return Commands.runOnce(() -> Intake.set(Speed));
    }

    public Command DriveElevator(double Speed) {
        if (DeadbandCheck(Speed) && MotorHeightBounds()) {
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