package frc.robot.Simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.*;
import org.ironmaple.simulation.seasonspecific.reefscape2025.*;

import com.pathplanner.lib.util.FlippingUtil;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;

public abstract class SimLogic {
    
    public static final Pose2d redHPCoralPose = new Pose2d(16.17, 1.33, new Rotation2d());
    public static final Pose2d blueHPCoralPose = FlippingUtil.flipFieldPose(redHPCoralPose);
    public static final double CORAL_LENGTH = Field.CORAL_LENGTH.in(Meters);

    public static boolean intakeHasCoral = false;
    public static boolean armHasCoral = false;
    public static boolean intakeHasAlgae = false;
    public static boolean armHasAlgae = false;
    public static int coralScored = 0;

    public static double armCoralPosition = -1;

    public static boolean robotHasCoral() {
        return intakeHasCoral || armHasCoral;
    }
    public static boolean robotHasAlgae(){
        return intakeHasAlgae || armHasAlgae;
    }
    public static void spawnHumanPlayerCoral() {
        spawnHumanPlayerCoral(Robot.isBlue());
    }

    public static void spawnHumanPlayerCoral(boolean blue) {
        if (!RobotContainer.MAPLESIM) {
            return;
        }
    }
    
}



// public static void guh() {
//     SimulatedArena.getInstance().simulationPeriodic();


//     SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
//     SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField (new Pose2d(2, 2, Rotation2d.fromDegrees(90))));


//     SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
//         //next practice (after 07/29/2025), gotta fix cuz aint no uses drimesimulation
//         driveSimulation.getSimulatonDriveTrainPose().getTranslation(), new Translation2d(0.35, 0),
//         swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
//         driveSimulation.getSimulatonDriveTrainPose().getRotation(), Meters.of(1.28), MetersPerSecond.of(2), Degrees.of(-35)));

// }