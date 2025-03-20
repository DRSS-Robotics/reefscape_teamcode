package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class CameraCommand extends Command {  
  //This file is the same as the subsystem code
 
  public PhotonCamera camera;
      private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
      private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    private final NetworkTable CameraTable = inst.getTable("CameraAlign");

          public CameraCommand() {
                camera = new PhotonCamera("April_RGB_Cam");
                System.out.println("works");
        /*  return camera/controller */
      }

     public class IntakeCommand extends Command {

     }

      public void readCamera() {
       // AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        boolean targetVisible = false;
        double targetYaw = 0.0;
        Transform3d targetOffset;
        double targetRange = 0.0;

        double closestRange = -1.0;
        double cameraPitch = 0.0;
        PhotonTrackedTarget closestTarget;
        double closestYaw = 0.0;
        Transform3d closestOffset;
        int closestID;
          DoublePublisher DataRange;
          DoublePublisher DataYaw;

        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size()-1);
            if (result.hasTargets()){
                for (var target : result.getTargets()){
                  //   if (target.getFiducialId() == 7)
                    int targetID = target.getFiducialId();
                    // if () {
                        /* Found Tag, record its information */
                        targetYaw = target.getYaw();
                        targetOffset = target.getBestCameraToTarget();                      
                        targetRange =
                        PhotonUtils.calculateDistanceToTargetMeters(
                            0.025, /*The height of the camera in meters */
                            0.22225, /*height of apriltag from ground to bottom */
                                Units.degreesToRadians(cameraPitch), 
                                Units.degreesToRadians(target.getPitch()));
        
                        targetVisible = true;
                        // checks if the target is the current closest target and then saves its values
                        if (targetRange < closestRange || closestRange == -1){
                          closestRange = targetRange;
                          closestTarget = target;
                          closestYaw = targetYaw;
                          closestOffset = targetOffset;
                          // the targetID is saved, since it may be useful to display to the driver
                          // in the future
                          closestID = targetID;
                        }
                        if (targetVisible) {
                            System.out.println("ID:");
                            System.out.println(targetID); /*print the ID */
                            System.out.println("Range:");
                            System.out.println(targetRange); /*print the range */
                            System.out.println("Yaw:");
                            System.out.println(targetYaw); /*print the yaw */
                            System.out.println("Offset:");
                            System.out.println(targetOffset);

                            DataRange = CameraTable.getDoubleTopic("targetYaw").publish();
                            DataYaw = CameraTable.getDoubleTopic("targetRange").publish();
                        
                            DataRange.set(targetYaw);
                            DataYaw.set(targetRange);
                                

                            }
                          }
                    }
          
                }
        }
      //  }
        // if (controller.getAButton() && targetVisible) {
        //     // System.out.println("ID:");
        //     // System.out.println(targetID);
        //     System.out.println("Range:");
        //     System.out.println(targetRange);
        //     System.out.println("Yaw:");
        //     System.out.println(targetYaw);
        //   }

              
            }
       
            

      
//  }