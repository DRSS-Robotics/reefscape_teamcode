package frc.robot.subsystems;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestHang extends SubsystemBase{
    public TestHang() {
        
    }




public Command HangUpCommand(DCMotor hangMotor, PWMMotorController hangMotorController, double hangSpeed) {
    return runOnce(() -> hangMotorController.set(hangSpeed));
}

public Command HangDownCommand(DCMotor hangMotor, PWMMotorController hangMotorController, double hangSpeed) {
    return runOnce(() -> hangMotorController.set(-hangSpeed));
}

}