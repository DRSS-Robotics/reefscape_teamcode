
package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorMechanism;


public class ElevatorDriveWithJoystick extends Command {

    private final ElevatorMechanism m_elevatorMechanism;
    CommandXboxController Joystick;


    public ElevatorDriveWithJoystick(ElevatorMechanism Elevator, CommandXboxController Controller) {
        m_elevatorMechanism = Elevator;
        Joystick = Controller;
        addRequirements(Elevator);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // m_elevatorMechanism.elevatorMotor.getClosedLoopController().setReference(-Joystick.getLeftY(), ControlType.kVelocity);
        System.out.println(Joystick.getLeftY());
    }
    
    @Override
    public void end(boolean Interrupted) {
        m_elevatorMechanism.elevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
