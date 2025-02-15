package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae_Mechanism;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Algae_Mechanism_Outake_Command extends Command{
    
    Algae_Mechanism algae_Mechanism;
    DCMotor spinnerWheelMotor;
    PWMMotorController spinnerWheelMotorController;
    double speedOfSpinnerWheels;
    
    public Algae_Mechanism_Outake_Command(Algae_Mechanism algae_Mechanism){
        algae_Mechanism = new Algae_Mechanism();
        XboxController gamepad2 = new XboxController(1);
        //Specifies that the command uses the algae subsystem. 
        //This stops other commands that require this subsystem from being run
        addRequirements(algae_Mechanism);
    }

    @Override
    public void execute(){
        //Set the speed of the spinner motor to 0.5
        algae_Mechanism.Algae_Outake_Command(spinnerWheelMotor, spinnerWheelMotorController, speedOfSpinnerWheels);
    }

    @Override
    public void initialize(){

    }

    public void end(boolean isInterrupted){
        //Default interrupted behavior is to cancel the current command and execute the new one
    }

    public boolean isFinished(){
        return false;
    }
}