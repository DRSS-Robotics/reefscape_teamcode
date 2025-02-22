// package frc.robot;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.ElevatorTakeTwo;

// public class ElevatorTakeTwo {
//     private MotorController motorController;
//     double currentPosition;
//     double targetPosition;

//     public ElevatorTakeTwo(MotorController motorController) {
//         this.motorController = motorController;
//     }

//         public void setControl(MotorRequest request) {
//         // Code to control the motor using the request
//         moveToPosition(request.getTargetPosition());
//     }

//     private void moveToPosition(double targetPosition) {
//         // Code to move the motor to the target position
//         // This is a simplified example; actual implementation would depend on your motor controller and sensors
//         currentPosition = targetPosition;
//         System.out.println("Motor moved to position: " + currentPosition);
    


//     public class MotorRequest {
//         private final double targetPosition;
    
//         public MotorRequest(double targetPosition) {
//             this.targetPosition = targetPosition;
//         }
    
//         public double getTargetPosition() {
//             return targetPosition;
//         }
//     }

//     public Command applyMotorRequest(Supplier<MotorRequest> requestSupplier) {
//         return new MoveMotorCommand(motorController, requestSupplier);
//     }
// }

//     private Command run(Runnable command) {
//         command.run();
//         return new Command();
//     }
// }

