package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.XboxController;

public class CameraSubsystem {
  public PhotonCamera camera;
  public PhotonTrackedTarget closestTarget;
  public double closestYaw = 0.0;
  public Transform3d closestOffset;
  public int closestID;
  public PhotonPipelineResult Result;
  public boolean ResultsAreEmpty;

  AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  // position of camera relative to robot origin
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(tagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
      
  public CameraSubsystem() {
    camera = new PhotonCamera("April_RGB_Cam");
    System.out.println("works");
    /* return camera/controller */
  }

  public void readCamera() {

    boolean targetVisible = false;
    double targetYaw = 0.0;
    Transform3d targetOffset;
    double targetRange = 0.0;

    double closestRange = -1.0;
    double cameraPitch = 0.0;
    var results = camera.getAllUnreadResults();

    ResultsAreEmpty = results.isEmpty();
    if (!results.isEmpty()) {
      Result = results.get(results.size() - 1);

      if (Result.hasTargets()) {

        for (var target : Result.getTargets()) {
          int targetID = target.getFiducialId();
          /* Found Tag, record its information */
          targetYaw = target.getYaw();
          targetOffset = target.getBestCameraToTarget();
          targetRange = PhotonUtils.calculateDistanceToTargetMeters(
              0.025, /* The height of the camera in meters */
              0.22225, /* height of apriltag from ground to bottom */
              Units.degreesToRadians(cameraPitch),
              Units.degreesToRadians(target.getPitch()));

          targetVisible = true;
          // checks if the target is the current closest target and then saves its values
          if (targetRange < closestRange || closestRange == -1) {

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
            System.out.println(targetID); /* print the ID */
            System.out.println("Range:");
            System.out.println(targetRange); /* print the range */
            System.out.println("Yaw:");
            System.out.println(targetYaw); /* print the yaw */
            System.out.println("Offset:");
            System.out.println(targetOffset); /* print the yaw */
          }
        }
        // print for testing the closest target values
        System.out.println("Closest Range:");
        System.out.println(closestRange);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("CameraData");

      }
    }

  }
}
