package frc.robot;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorFinal {
    private MotorController motorController;

    public ElevatorFinal(MotorController motorController) {
        this.motorController = motorController;
    }

    public Command applyMotorRequest(Supplier<MotorRequest> requestSupplier) {
        return new MoveMotorCommand(motorController, requestSupplier);
    }
}