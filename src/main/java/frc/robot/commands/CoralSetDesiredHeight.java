package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class CoralSetDesiredHeight extends Command {

    CoralMechanism CM;
    int DH;


    public CoralSetDesiredHeight(CoralMechanism Coral, int Height) {
        CM = Coral;
        DH = Height;
        // addRequirements(Coral);
    }
    
    @Override
    public void initialize() {
        CM.DesiredCoralHeight = DH;
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
