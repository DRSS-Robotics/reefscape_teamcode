package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class CoralMoveToIndex extends Command {

    CoralMechanism CM;
    int DH;


    public CoralMoveToIndex(CoralMechanism Coral, int NewHeight) {
        CM = Coral;
        DH = NewHeight;
        addRequirements(Coral);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        CM.Elevator.getClosedLoopController().setReference(CM.CoralHeights[DH], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        CM.Elevator.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(CM.Elevator.getEncoder().getPosition() -
      CM.CoralHeights[DH]) < 0.25);
    }
}
