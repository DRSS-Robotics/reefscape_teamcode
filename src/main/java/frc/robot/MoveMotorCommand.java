package frc.robot;


import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;


public class MoveMotorCommand extends Command{
    private final MotorController motorController;
    private final Supplier<MotorRequest> requestSupplier;

    public MoveMotorCommand(MotorController motorController, Supplier<MotorRequest> requestSupplier) {
        this.motorController = motorController;
        this.requestSupplier = requestSupplier;
    }

    @Override
    public void initialize() {
        // Initialize any necessary variables or states
    }

    @Override
    public void execute() {
        // Get the request and move the motor
        MotorRequest request = requestSupplier.get();
        motorController.setControl(request);
    }

    @Override
    public boolean isFinished() {
        // Determine if the command is finished (e.g., motor has reached the target position)
        return true; // Simplified for example purposes
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up if necessary
    }
}
