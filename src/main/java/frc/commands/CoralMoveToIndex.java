package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorMechanism;

public class CoralMoveToIndex extends Command {

    elevatorMechanism coralElevatorMechanism;
    int desiredHeight;


    public CoralMoveToIndex(elevatorMechanism Coral, int NewHeight) {
        coralElevatorMechanism = Coral;
        desiredHeight = NewHeight;
        addRequirements(Coral);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        coralElevatorMechanism.elevatorMechanism.getClosedLoopController().setReference(coralElevatorMechanism.CoralHeights[desiredHeight], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        coralElevatorMechanism.elevatorMechanism.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(coralElevatorMechanism.elevatorMechanism.getEncoder().getPosition() -
      coralElevatorMechanism.CoralHeights[desiredHeight]) < 0.25);
    }
}