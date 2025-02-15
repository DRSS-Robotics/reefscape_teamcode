package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae_Mechanism;
import edu.wpi.first.wpilibj.XboxController;

public class Algae_Mechanism_Down_Command extends Command{
    Algae_Mechanism algaeMechanism;

    public Algae_Mechanism_Down_Command(Algae_Mechanism algaeMechanism){
        Algae_Mechanism algae_Mechanism = new Algae_Mechanism();
        //XboxController gamepad2 = new XboxController(1);
        //Specifies that the command uses the algae subsystem. 
        //This stops other commands that require this subsystem from being run
        addRequirements(this.algaeMechanism);
    }

    @Override
    public void execute(){
        
       
    }

    @Override
    public void initialize(){

    }

    public void end(boolean isInterrupted){
        //Default interrupted behavior is to cancel the current command and execute the new one
    }

    public boolean isFinished(){
        if(!algaeMechanism.isUp(null)){
            return true;
        } else {
            return false;
        }
    }
}