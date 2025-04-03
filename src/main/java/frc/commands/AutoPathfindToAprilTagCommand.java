package frc.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;

public class AutoPathfindToAprilTagCommand extends Command {

    CameraSubsystem Cam;
    Pose2d TargetPos;
    Rotation2d DesiredRotation;
    // false = left of reef, true = right of reef
    boolean Direction;

    public AutoPathfindToAprilTagCommand(CameraSubsystem Camera, boolean ReefDirection) {
        Cam = Camera;
        Direction = ReefDirection;
    }

    @Override
    public void initialize() {
        System.out.println(Cam.closestID);

        switch (Cam.closestID) {
            case 7:
                DesiredRotation = new Rotation2d(180.0);
                TargetPos = Direction ? new Pose2d(14.381, 3.781, DesiredRotation)
                        : new Pose2d(14.381, 4.09, DesiredRotation);
                break;

            case 9:
                DesiredRotation = new Rotation2d(-60.0);
                TargetPos = Direction ? new Pose2d(12.636, 5.302, DesiredRotation)
                        : new Pose2d(12.363, 5.136, DesiredRotation);
                break;

            case 11:
                DesiredRotation = new Rotation2d(60.0);
                TargetPos = Direction ? new Pose2d(12.168, 3.018, DesiredRotation)
                        : new Pose2d(12.443, 2.862, DesiredRotation);
                break;

            default:
                TargetPos = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
                break;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                TargetPos);

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, DesiredRotation));

        path.preventFlipping = true;
    }

}
