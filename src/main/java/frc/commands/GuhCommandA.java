package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.guh;


public class GuhCommandA extends Command {
    
    guh guh;

    public GuhCommandA(guh Guh) {
        addRequirements(Guh);
        this.guh = Guh;
    }

    @Override
    public void initialize() {
        guh.hangMotor.getClosedLoopController().setReference(Constants.guhHangUp, ControlType.kPosition);
    }
    @Override
    public void execute() {
        System.out.println("guh");
    }
    

}

