package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.XboxController;


public class CameraSubsystem {  
      public PhotonCamera camera;
      private XboxController controller;
      
          public CameraSubsystem() {
                camera = new PhotonCamera("April_RGB_Cam");
              controller = new XboxController(0);
                System.out.println("works");
        /*  return camera/controller */
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
                        if (controller.getAButton() && targetVisible) {
                            System.out.println("ID:");
                            System.out.println(targetID); /*print the ID */
                            System.out.println("Range:");
                            System.out.println(targetRange); /*print the range */
                            System.out.println("Yaw:");
                            System.out.println(targetYaw); /*print the yaw */
                            System.out.println("Offset:");
                            System.out.println(targetOffset); /*print the yaw */
                          }
                    }
                    // print for testing the closest target values
                    System.out.println("Closest Range:");
                    System.out.println(closestRange);
                    NetworkTableInstance inst = NetworkTableInstance.getDefault();
                    NetworkTable table = inst.getTable("CameraData");
                                   
          
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

            if(controller.getBButton() && targetVisible) {
              System.out.println("dijzgh.gkcvjoz.xkchfuv");
              System.out.println(targetRange - targetYaw);
              if(targetRange == 0) 
              if(targetRange == 0) {
                System.out.println("You are there");
              }
              else if(targetRange > 0 && targetYaw > 0) {

              }
              
            }
       
            
      }
      
    }
