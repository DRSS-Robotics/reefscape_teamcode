package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class CoralMoveToIndex extends Command {

    CoralMechanism CM;
    int DH;


    public CoralMoveToIndex(CoralMechanism Coral) {
        CM = Coral;
        addRequirements(Coral);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        if (!isFinished()) {
            CM.Elevator.getClosedLoopController().setReference(CM.CoralHeights[CM.DesiredCoralHeight], ControlType.kPosition);
        } else {
            end(false);
        }
    }
    
    @Override
    public void end(boolean Interrupted) {
        CM.Elevator.stopMotor();
    }

    @Override
    public boolean isFinished() {
        System.out.println("Guhh");
      return (Math.abs(CM.Elevator.getEncoder().getPosition() -
      CM.CoralHeights[CM.DesiredCoralHeight]) < 0.25);
    }
}
