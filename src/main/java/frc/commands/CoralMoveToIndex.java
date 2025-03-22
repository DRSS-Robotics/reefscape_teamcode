package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorMechanism;

public class CoralMoveToIndex extends Command {

    private final elevatorMechanism m_elevatorMechanism;
    int desiredHeight;


    public CoralMoveToIndex(elevatorMechanism Coral, int NewHeight) {
        m_elevatorMechanism = Coral;
        desiredHeight = NewHeight;
        addRequirements(Coral);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        m_elevatorMechanism.elevatorMechanism.getClosedLoopController().setReference(m_elevatorMechanism.CoralHeights[desiredHeight], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        m_elevatorMechanism.elevatorMechanism.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(m_elevatorMechanism.elevatorMechanism.getEncoder().getPosition() -
      m_elevatorMechanism.CoralHeights[desiredHeight]) < 0.25);
    }
}