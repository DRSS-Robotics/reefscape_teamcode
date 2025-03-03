package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class coralElevatorCommand extends Command {
    private Coral_Mechanism coralMechanism;
    private RobotContainer robotContainer;

    public coralElevatorCommand(Coral_Mechanism coralMechanism, RobotContainer robotContainer) {
        coralMechanism = new Coral_Mechanism();
        robotContainer = new RobotContainer();

        addRequirements(coralMechanism);
    }


@Override
public void initialize() {
    coralMechanism.startCoralElevatorAction(robotContainer.joystick2);
}

@Override
public boolean isFinished() {
    if(coralMechanism.isCoralActivated(robotContainer.joystick2)) {
        return false;
    }
    else {
        return true;
    }
}
}
