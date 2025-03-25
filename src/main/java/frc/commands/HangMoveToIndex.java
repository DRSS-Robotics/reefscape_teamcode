package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HangMechanism;

public class HangMoveToIndex extends Command {

    private final HangMechanism m_HangMechanism;
    int desiredRotations;


    public HangMoveToIndex(HangMechanism Hang, int NewHeight) {
        m_HangMechanism = Hang;
        desiredRotations = NewHeight;
        addRequirements(Hang);
    }
    
    @Override
    public void initialize() {
        // System.out.println(desiredRotations);
        // System.out.println("Guhh");
    }
    
    @Override
    public void execute() {
        m_HangMechanism.HangMotor.getClosedLoopController()
        .setReference(Constants.kHangTargetRotations[desiredRotations], ControlType.kPosition);
    }
    
    @Override
    public void end(boolean Interrupted) {
        m_HangMechanism.HangMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return (Math.abs(m_HangMechanism.HangMotor.getEncoder().getPosition() -
      Constants.kHangTargetRotations[desiredRotations]) < 0.125);
    }
}