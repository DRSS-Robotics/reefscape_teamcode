package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
      public PhotonPoseEstimator photonEstimator;
      
          public CameraSubsystem() {
                camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
                camera.setPipelineIndex(0);
                controller = new XboxController(0);
                System.out.println("works");

                //add photonposeestimator 09/09/2025
                final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
                Transform3d cameraPosition = new Transform3d(new Translation3d(0,0.025,0), new Rotation3d(0,0,0));
                photonEstimator = new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraPosition);
               
        /*  return camera/controller */
      }

      
      public void readCamera() {
       //AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        boolean targetVisible = false;
        double targetYaw = 0.0;
        Transform3d targetOffset;
        double targetRange = 0.0;
        // nathan was here
        double closestRange = -1.0;
        double cameraPitch = 0.0;
        PhotonTrackedTarget closestTarget;
        double closestYaw = 0.0;
        Transform3d closestOffset;
        int closestID;

        
        Optional<EstimatedRobotPose> visionEst = Optional.empty(); //create arraylist variable for estimated postion based on photon pose estimator
        var results = camera.getAllUnreadResults();
        
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size()-1);
            if (result.hasTargets()){
                //for (var target : result.getTargets()){
                  //   if (target.getFiducialId() == 7)
                  visionEst = photonEstimator.update(result); //updates pose  Mokka was here:)
                  PhotonTrackedTarget target = result.getBestTarget();

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
                            System.out.println("Photon Pose Estimate:");
                            System.out.println(visionEst); /*print pose estimator changes */
                            camera.takeOutputSnapshot();
                          }
                    }
                    // print for testing the closest target values

                    NetworkTableInstance inst = NetworkTableInstance.getDefault();
                    NetworkTable table = inst.getTable("CameraData");
                    
                                   
          
                }
        }   
      }
    //}
