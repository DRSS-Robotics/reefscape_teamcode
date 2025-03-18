package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
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
    boolean JoystickDrive;
    boolean IsAtTarget = false;
    boolean ShouldGoUp = false;
    int DesiredCoralHeight;
    double[] CoralHeights = { 15.0, 15.0, 110.0, 110.0 };

    boolean PrevElevDir = true;

    public final double Deadband = 0.04;

    public CoralMechanism(int ElevID, int IntakeID, CommandXboxController Controller, boolean DriveWithJoystick) {
        Elevator = new SparkMax(ElevID, MotorType.kBrushless);
        // Elevator.getEncoder().setPosition(0);
        // elevator encoder is relative
        Intake = new SparkMax(IntakeID, MotorType.kBrushless);
        // System.out.println(Elevator.getEncoder().getClass());
        TeleController = Controller;
        JoystickDrive = DriveWithJoystick;
    }

    public boolean DeadbandCheck(double Value) {
        // System.out.println(Math.abs(Value) > Deadband);
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds(double Speed) {
        if (Elevator.getEncoder().getPosition() - 8 * Speed >= 120 && Speed > 0) {
            return false;
        } else if (Elevator.getEncoder().getPosition() - 8 * Speed < 5 && Speed < 0) {
            return false;
        } else {
            return true;
        }

    }

    public Command SetIntakeSpeed(double Speed) {
        return Commands.runOnce(() -> Intake.set(Speed));
    }

    public Command DriveWithMotorSpeed(double Speed) {
        if (DeadbandCheck(Speed) && MotorHeightBounds(Speed)) {
            Elevator.set(Speed);
        } else {
            System.out.println("Guhhhhh");
            Elevator.stopMotor();
        }
        return Commands.none();
    }

    public Command DriveWithJoystick(CommandXboxController Controller) {
        if (DeadbandCheck(Controller.getLeftY()) && MotorHeightBounds(Controller.getLeftY())) {
            Elevator.set(-Controller.getLeftY());
        } else {
            Elevator.stopMotor();
        }
        return Commands.none();
    }

    public Command SetDesiredElevatorHeight(int CoralLevelIndex) {
        IsAtTarget = false;
        DesiredCoralHeight = CoralLevelIndex;
        System.out.println("Guhh");
        return Commands.none();
    }

    // public Command DriveToPosition() {
    //     ShouldGoUp = (Elevator.getEncoder().getPosition() < CoralHeights[DesiredCoralHeight]);

    //     if (PrevElevDir != ShouldGoUp) {
    //         IsAtTarget = true;
    //         Elevator.stopMotor();
    //         System.out.println("reached target");
    //     }

    //     if (!IsAtTarget) {
    //         if (ShouldGoUp) {
    //             DriveWithMotorSpeed(0.6);
    //             // System.out.println("up");
    //         } else {
    //             DriveWithMotorSpeed(-0.6);
    //             // System.out.println("down");
    //         }
    //     }

    //     PrevElevDir = ShouldGoUp;

    //     return Commands.none();
    // }
    public Command DriveToPosition(double InPos) {
        // ShouldGoUp = (Elevator.getEncoder().getPosition() < CoralHeights[DesiredCoralHeight]);
        Elevator.getClosedLoopController().setReference(InPos, ControlType.kPosition);
        return Commands.none();
    }

    @Override
    public void periodic() {
        Command CurrentState = JoystickDrive ? DriveWithJoystick(TeleController) : DriveToPosition(CoralHeights[DesiredCoralHeight]);
    }
}