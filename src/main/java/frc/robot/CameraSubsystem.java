package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;


public class CameraSubsystem {  
      public PhotonCamera camera;
      private XboxController controller;
      
          public CameraSubsystem() {
                camera = new PhotonCamera("April_RGB_Cam");
              controller = new XboxController(0);
                System.out.println("works");
        //   return camera/controller
      }


      public void readCamera() {
        AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size()-1);
                if (result.hasTargets()){
                    for (var target : result.getTargets()){
                     //   if (target.getFiducialId() == 7)
                        int targetID = target.getFiducialId();
                        // if () {
                            // Found Tag, record its information
                            targetID = target.getFiducialId();
                            targetYaw = target.getYaw();
                            targetRange =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                0.025, //The height of the camera in meters
                                0.22225, //height of apriltag from ground to bottom
                                    Units.degreesToRadians(0.0), 
                                    Units.degreesToRadians(target.getPitch()));
            
                            targetVisible = true;
                            if (controller.getAButton() && targetVisible) {
                                System.out.println("ID:");
                                System.out.println(targetID); //print the ID
                                System.out.println("Range:");
                                System.out.println(targetRange); //print the range
                                System.out.println("Yaw:");
                                System.out.println(targetYaw); //print the yaw
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


        //TEST NOT PART OF THE CODE FOR REVIEW
        if (controller.getYButton() && targetVisible) {
            turn =
                    (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
            forward =
                    (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.kMaxLinearSpeed;
        }
            
      }
    }
