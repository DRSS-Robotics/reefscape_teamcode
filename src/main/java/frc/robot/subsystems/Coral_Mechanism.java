package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Coral_Mechanism extends SubsystemBase {

    private boolean coralActivate;

    public static final SparkMax coralElevator = new SparkMax(13, MotorType.kBrushless);  // PWM Port 0 for Elevator motor
  

    public Coral_Mechanism() {
        coralActivate = false;
    }

    //CHeck if the joystick is in use. If it is, the coral is activated.
    public boolean isCoralActivated(CommandXboxController joystick2) {
        // Check if the joystick is moved significantly in the Y-axis
        if (joystick2.getLeftY() > 0.04 || joystick2.getLeftY() < -0.04) {
            //Joystick in use
            coralActivate = true;
        } else {
            //Joystick not in use
            coralActivate = false;
        }
        return coralActivate;
    }

    //Start an elevator action depending on whether we are pushing up or down
    public void startCoralElevatorAction(CommandXboxController joystick2) {
        // Control the elevator motor based on joystick Y-axis
        if (coralActivate) {
            if (joystick2.getLeftY() < -0.4) {
                // Move the elevator up
                runOnce(() -> coralElevator.set(0.75));
            
            } else if (joystick2.getLeftY() > 0.4) {
                // Move the elevator down
                runOnce(() -> coralElevator.set(-0.75));
            }
        } else{
            runOnce(() -> coralElevator.set(0.0));
        }
        // Stop the elevator if not activated
    }
}
