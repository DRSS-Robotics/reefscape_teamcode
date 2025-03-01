package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

public class CoralElevatorUpCommand extends Command{

    public CoralElevatorUpCommand(){
        Coral_Mechanism coralMechanism = new Coral_Mechanism();
        RobotContainer robotContainer = new RobotContainer();
        addRequirements(coralMechanism);
    }

    @Override
    public void execute(){

    }

    @Override
    public void initialize(){

    }

    public void end(boolean isInterrupted){

    }

    public boolean isFinished(Coral_Mechanism coral_Mechanism, CommandXboxController joystick2, RobotContainer robotContainer){
        if (robotContainer.joystick2.getLeftY() > -0.04) {
            coral_Mechanism.coralActivate = true;
        } else{
         coral_Mechanism.coralActivate = false;
        }
        return coral_Mechanism.coralActivate;
        }
    }
