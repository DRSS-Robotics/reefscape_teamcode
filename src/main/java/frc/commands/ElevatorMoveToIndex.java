package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorMechanism;

public class ElevatorMoveToIndex extends Command {

    private final ElevatorMechanism m_elevatorMechanism;
    double DesiredHeight;

    public ElevatorMoveToIndex(ElevatorMechanism Elevator, int NewHeight) {
        m_elevatorMechanism = Elevator;
        DesiredHeight = m_elevatorMechanism.IsAtComp ? Constants.kCompElevatorTargetHeights[NewHeight]
                : Constants.kPracticeElevatorTargetHeights[NewHeight];
        addRequirements(Elevator);
    }

    @Override
    public void initialize() {
        System.out.println(DesiredHeight);
        // System.out.println("Guhh");
    }

    @Override
    public void execute() {
        m_elevatorMechanism.elevatorMotor.getClosedLoopController()
                .setReference(DesiredHeight, ControlType.kPosition);
    }

    @Override
    public void end(boolean Interrupted) {
        m_elevatorMechanism.elevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_elevatorMechanism.elevatorMotor.getEncoder().getPosition() -
                DesiredHeight) < 0.125);
    }
}