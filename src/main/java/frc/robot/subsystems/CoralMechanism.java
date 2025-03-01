package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class CoralMechanism extends SubsystemBase {

    public boolean ElevatorActivated = false;
    public double Deadband = 0.04;
    public int MinElevEncoderHeight = 0;
    public int MaxElevEncoderHeight = 3;

    public CoralMechanism() {
        SparkMax Elevator = new SparkMax(0 /* TODO: replace w/ elevator motor id */, MotorType.kBrushless);
        SparkMax Intake = new SparkMax(0 /* TODO: replace w/ intake motor id */, MotorType.kBrushless);
    }

    public void SetState(NewState) {
        ElevatorActivated = NewState;
        if (!NewState) {
            Elevator.set(0);
        }
    }

    public Command SetIntakeSpeed(double Speed) {
        return Commands.runOnce(() -> Intake.set(Speed));
    }

    public Command DriveElevator(CommandXboxController Controller) {
        if (ElevatorActivated) {
            return Commands.runOnce(() -> Elevator.set(-0.5 * Controller.getLeftY()));
        } else {
            return Commands.runOnce(() -> Elevator.set(0));
        }
       }
   }