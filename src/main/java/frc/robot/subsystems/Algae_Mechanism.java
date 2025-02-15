package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

public class Algae_Mechanism extends SubsystemBase{
    public Algae_Mechanism() {

        XboxController gamepad2 = new XboxController(1); //move to robot container
        double speedOfSpinnerWheels = 0.5;
        //Fix this later
        Encoder pivotMotorEncoder = new Encoder(0,0);
        
    }

//     public static void spinnerPower(XboxController gamepad2, DCMotor spinnerWheelMotor, PWMMotorController spinnerWheelMotorController) {
//         boolean isRightBumperPressed = gamepad2.getRightBumperPressed();//move to robot container
//         double speedOfSpinnerWheels = 0.5;
// //Change speed in the command file, speed is changed based off of if the bumper is pressed
//         if(isRightBumperPressed) {
//             //Set speed to 0.5 if bumper is pressed
//             spinnerWheelMotorController.set(speedOfSpinnerWheels);
//         }
//         else {
//             //Do not set a speed if bumper is not pressed
//             spinnerWheelMotorController.set(speedOfSpinnerWheels);
//         }
//     }
    public static boolean isUp(Encoder pivotMotorEncoder) {
        boolean isUp = true;
        double pivotEncoderValue = pivotMotorEncoder.get();

        if (9 < pivotEncoderValue && pivotEncoderValue < 10) {
            isUp = true;
        } else if (0 < pivotEncoderValue && pivotEncoderValue < 1) {
            //Why do we have this else if?
            isUp = false;
        } else{
            isUp = false;
        }
        return isUp;
    }
    public static void pivotMotion(XboxController gamepad2, DCMotor pivotMotor, PWMMotorController pivotMotorController, boolean isUp, int pivotRemainingDistance, Encoder pivotMotorEncoder) {
        // boolean isXButtonPressed = gamepad2.getXButtonPressed();
        // double speedOfPivotMotor = 0.5;

        // if (isXButtonPressed && isUp && pivotRemainingDistance > 0) {
        //     //Reverse speed to go down instead of up
        //     pivotMotorController.set(-speedOfPivotMotor);
        // } else if(isXButtonPressed && !isUp && pivotRemainingDistance > 0) {
        //     pivotMotorController.set(speedOfPivotMotor);
        // } else{
        //     pivotMotorEncoder.reset();
        // }
    }
    public static int pivotRemainingDistance(Encoder pivotMotorEncoder) {
        int pivotTargetPosition = 9;
        int pivotRemainingDistance = pivotTargetPosition - pivotMotorEncoder.get();
        return pivotRemainingDistance;
    }

    public Command Algae_Intake_Command(DCMotor spinnerWheelMotor, PWMMotorController spinnerWheelMotorController, double speedOfSpinnerWheels ) {
        return runOnce(() -> spinnerWheelMotorController.set(speedOfSpinnerWheels));

        }
    public Command Algae_Outake_Command(DCMotor spinnerWheelMotor, PWMMotorController spinnerWheelMotorController, double speedOfSpinnerWheels ) {
            return runOnce(() -> spinnerWheelMotorController.set(-speedOfSpinnerWheels));
    
        }
    }



