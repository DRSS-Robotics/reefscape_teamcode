package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class coralElevatorCommand extends Command implements Runnable {
    private Coral_Mechanism coralMechanism;
    private RobotContainer robotContainer;

    public coralElevatorCommand() {
        coralMechanism = new Coral_Mechanism();
        robotContainer = new RobotContainer();

        addRequirements(coralMechanism);
    }


@Override
public void initialize() {
    //Start the elevator action when the command is scheduled
    coralMechanism.startCoralElevatorAction(robotContainer.joystick2);
}

@Override
public boolean isFinished() {
    //If the joystick is inactive, the command is done
    if(coralMechanism.isCoralActivated(robotContainer.joystick2)) {
        return false;
    }
    else {
        return true;
    }
}


@Override
public void run() {
    coralMechanism.startCoralElevatorAction(robotContainer.joystick2);
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'run'");
}
}
