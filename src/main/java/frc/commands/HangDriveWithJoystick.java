
package frc.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.HangMechanism;


public class HangDriveWithJoystick extends Command {

    private final HangMechanism m_hangMechanism;
    CommandXboxController Joystick;


    public HangDriveWithJoystick (HangMechanism Hang, CommandXboxController Controller) {
        m_hangMechanism = Hang;
        Joystick = Controller;
        addRequirements(Hang);
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
        m_hangMechanism.HangMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
