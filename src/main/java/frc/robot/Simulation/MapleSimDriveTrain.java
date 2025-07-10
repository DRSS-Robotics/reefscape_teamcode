package frc.robot.Simulation;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class MapleSimDriveTrain {
    private SwerveDriveSimulation swerveDriveSimulation;
    // Create and configure a drivetrain simulation configuration
    final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
    // Specify gyro type (for realistic gyro drifting and error simulation)
    .withGyro(COTS.ofPigeon2())
    // Specify swerve module (for realistic swerve dynamics)
    .withSwerveModule(COTS.ofMark4(
            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
            DCMotor.getKrakenX60(1), // Steer motor is a kraken x60
            COTS.WHEELS.BLUE_NITRILE_TREAD.cof, // uses the andymark blue nitrile tread (cloese type to the wheels we have)
            3)) // L3 Gear ratio (may need to be changed)
    // Configures the track length and track width (spacing between swerve modules)
    .withTrackLengthTrackWidth(Inches.of(17), Inches.of(17))
    // Configures the bumper size (dimensions of the robot bumper)
    .withBumperSize(Inches.of(36), Inches.of(36));
    
    
        public MapleSimDriveTrain () {
            /* Create a swerve drive simulation */
    this.swerveDriveSimulation = new SwerveDriveSimulation(
        // Specify Configuration
        driveTrainSimulationConfig,
        // Specify starting pose
        new Pose2d(3, 3, new Rotation2d())
);
// Register the drivetrain simulation to the default simulation world
SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }
}