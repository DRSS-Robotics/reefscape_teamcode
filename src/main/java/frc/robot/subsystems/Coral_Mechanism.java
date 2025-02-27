package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class Coral_Mechanism extends SubsystemBase{

    public boolean coralActivate;

    public Coral_Mechanism(){
        RobotContainer robotContainer = new RobotContainer();
       SparkMax coralElevator = new SparkMax(0, MotorType.kBrushless);
        coralActivate = false;

    }
    public boolean isCoralActivated (CommandXboxController joystick2) {
        if (joystick2.getLeftY() > 0.04) {
           coralActivate = true;
       } else if (joystick2.getLeftY() < -0.04) {
           coralActivate = true;
       } else{
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
   }s
}