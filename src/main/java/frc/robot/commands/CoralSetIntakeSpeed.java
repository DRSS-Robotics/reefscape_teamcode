package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class CoralSetIntakeSpeed extends Command {

    CoralMechanism CM;
    double NewSpeed;


    public CoralSetIntakeSpeed(CoralMechanism Coral, Double Speed) {
        CM = Coral;
        NewSpeed = Speed;
        // addRequirements(Coral);
    }
    
    @Override
    public void initialize() {
        CM.Intake.set(NewSpeed);;
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
