package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Coral_Mechanism extends SubsystemBase {

public class Coral_Mechanism extends SubsystemBase{

    public static final SparkMax coralElevator = new SparkMax(0, MotorType.kBrushless);  // PWM Port 0 for Elevator motor
    public static final SparkMax coralIntake = new SparkMax(1, MotorType.kBrushless);     // PWM Port 1 for Intake motor

    public Coral_Mechanism() {
        coralActivate = false;
    }

    public boolean isCoralActivated(CommandXboxController joystick2) {
        // Check if the joystick is moved significantly in the Y-axis
        if (joystick2.getLeftY() > 0.04 || joystick2.getLeftY() < -0.04) {
            coralActivate = true;
        } else {
            coralActivate = false;
        }
        return coralActivate;
    }

   public Command startCoralElevatorAction(CommandXboxController joystick2, SparkMax coralElevator) {
       if ((coralActivate) && (joystick2.getLeftY() < 0)){
           //We want to go up
           return runOnce(() -> coralElevator.set(0.5));
       } else if ((coralActivate) && (joystick2.getLeftY() > 0)){
           //We want to go down
           return runOnce(() -> coralElevator.set(-0.5));
       }
           else {
            return runOnce(() -> coralElevator.set(0.0));
       }
   }
}