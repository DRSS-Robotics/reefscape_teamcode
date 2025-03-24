package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralMechanism extends SubsystemBase {

    SparkMax Elevator;
    SparkMax Intake;
    CommandXboxController TeleController;

    public final double Deadband = 0.04;
    public CoralMechanism(int ElevID, int IntakeID, CommandXboxController Controller) {
        Elevator = new SparkMax(ElevID, MotorType.kBrushless);
        // Elevator.getEncoder().setPosition(0);
        // elevator encoder is relative
        Intake = new SparkMax(IntakeID, MotorType.kBrushless);
        //System.out.println(Elevator.getEncoder().getClass());
        TeleController = Controller;
    }

    public boolean DeadbandCheck(double Value) {
        //System.out.println(Math.abs(Value) > Deadband);
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds(double ControllerY) {
        // in meters
        double MotorPositionLinear =
        Elevator.getEncoder().getPosition() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        // also in meters - what else did you think it would be in?
        double MotorVelocityLinear =
        Elevator.getEncoder().getVelocity() * Constants.kElevatorDrumRadius * 2 * Math.PI;

        // System.out.println(MotorPositionLinear);
        // if (MotorPositionLinear + MotorVelocityLinear >= Constants.kMaxElevatorHeightMeters ||
        //     MotorPositionLinear + MotorVelocityLinear < Constants.kMinElevatorHeightMeters) {
        if (Elevator.getEncoder().getPosition() - 8 * ControllerY < 5 ||
            Elevator.getEncoder().getPosition() - 8 * ControllerY >= 120) {
            return false;
        } else {
            return true;
        }
    }

    public Command SetIntakeSpeed(double Speed) {
        return Commands.runOnce(() -> Intake.set(Speed));
    }

    public Command DriveElevator(double Speed) {
        if (DeadbandCheck(Speed) && MotorHeightBounds(Speed)) {
            return Commands.runOnce(() -> Elevator.set(-Speed));
        } else {
            return Commands.runOnce(() -> Elevator.stopMotor());
        }
    }

    public Command DriveElevator(CommandXboxController Controller) {
        if (DeadbandCheck(Controller.getLeftY()) && MotorHeightBounds(Controller.getLeftY())) {
            Elevator.set(-Controller.getLeftY());
            return Commands.run(() -> Elevator.set(-Controller.getLeftY()));
        } else {
            Elevator.stopMotor();
            return Commands.run(() -> Elevator.stopMotor());
        }
    }

    @Override
    public void periodic() {
        DriveElevator(TeleController);
    }
   }