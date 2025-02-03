package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Algae_Mechanism;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.XboxController;

public class Algae_Mechanism_Spinner_Command extends Command{
    Algae_Mechanism algaeMechanism;
    private DoubleSupplier forward;
    private DoubleSupplier reverse;
    XboxController gamepad2;

    public Algae_Mechanism_Spinner_Command(DoubleSupplier forward, DoubleSupplier reverse, Algae_Mechanism algaeSubsystem){
        this.forward = forward;
        this.reverse = reverse;
        this.algaeMechanism = algaeMechanism;
        XboxController gamepad2 = new XboxController(1);
        Trigger rightBumper = new JoystickButton(exampleController, XboxController.rightBumper.isRightBumperPressed);

        //Specifies that the command uses the algae subsystem. 
        //This stops other commands that require this subsystem from being run
        addRequirements(this.algaeMechanism);
    }

    @Override
    public void execute(){
        //Set the speed of the spinner motor to 0.5
        algaeMechanism.spinnerWheelMotorController.set(algaeMechanism.speedOfSpinnerWheels);
    }

    @Override
    public void init(){

    }

    public void end(boolean isInterrupted){
        //Default interrupted behavior is to cancel the current command and execute the new one
    }

    public void isFinished(){

    }
}