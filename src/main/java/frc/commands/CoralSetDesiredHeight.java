package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorMechanism;

public class CoralSetDesiredHeight extends Command {

    private final elevatorMechanism m_elevatorMechanism;
    int desiredHeight;


    public CoralSetDesiredHeight(elevatorMechanism Coral, int Height) {
        m_elevatorMechanism = Coral;
        desiredHeight = Height;
        // addRequirements(Coral);
    }
    
    @Override
    public void initialize() {
        m_elevatorMechanism.DesiredCoralHeight = desiredHeight;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean Interrupted) {
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}