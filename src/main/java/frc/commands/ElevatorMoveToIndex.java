
package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorMechanism;

public class ElevatorMoveToIndex extends Command {

    private final ElevatorMechanism m_elevatorMechanism;
    int desiredHeight;


    public ElevatorMoveToIndex(ElevatorMechanism Elevator, int NewHeight) {
        m_elevatorMechanism = Elevator;
        desiredHeight = NewHeight;
        addRequirements(Elevator);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        m_elevatorMechanism.elevatorMotor.getClosedLoopController().setReference(Constants.kElevatorTargetHeights[desiredHeight], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        m_elevatorMechanism.elevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(m_elevatorMechanism.elevatorMotor.getEncoder().getPosition() -
      Constants.kElevatorTargetHeights[desiredHeight]) < 0.125);
    }
}
