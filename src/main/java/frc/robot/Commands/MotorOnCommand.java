package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.CommandHangSubsystem;

public class MotorOnCommand extends Command {
    private final CommandHangSubsystem motorSubsystem;

    public MotorOnCommand(CommandHangSubsystem motorSubsystem) {
        this.motorSubsystem = motorSubsystem;
        addRequirements(motorSubsystem); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        motorSubsystem.setSpeed(0.5
        ); // Set motor speed to full speed
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        motorSubsystem.setSpeed(0.0); // Stop the motor
    }

    @Override
    public boolean isFinished() {
        return false; // Keep running until interrupted
    }
}
