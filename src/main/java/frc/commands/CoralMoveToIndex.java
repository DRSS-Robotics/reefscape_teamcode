package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorMechanism;

public class CoralMoveToIndex extends Command {

    private final ElevatorMechanism m_elevatorMechanism;
    int desiredHeight;


    public CoralMoveToIndex(ElevatorMechanism Coral, int NewHeight) {
        m_elevatorMechanism = Coral;
        desiredHeight = NewHeight;
        addRequirements(Coral);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        m_elevatorMechanism.elevatorMotor.getClosedLoopController().setReference(m_elevatorMechanism.CoralHeights[desiredHeight], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        m_elevatorMechanism.elevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(m_elevatorMechanism.elevatorMotor.getEncoder().getPosition() -
      m_elevatorMechanism.CoralHeights[desiredHeight]) < 0.25);
    }
}