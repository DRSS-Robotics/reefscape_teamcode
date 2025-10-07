package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorMechanism;
import frc.robot.subsystems.HangMechanism;
import frc.robot.subsystems.CoralMechanism.CoralMechanism;
import frc.commands.CoralIntakeCommand;
import frc.commands.CoralAutoIntakeCommand;
import frc.commands.CoralOuttakeCommand;
import frc.commands.CoralAutoOuttakeCommand;
import frc.commands.CoralStopCommand;
import frc.commands.CoralAutoStopCommand;
import frc.commands.ElevatorMoveToIndex;
import frc.commands.HangMoveToIndex;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
        //Having this true will use maplesim in the simulation and setting it to false will disable maplesim in the simulation 
    private static final boolean USE_MAPLESIM = true;
    public static final boolean MAPLESIM = USE_MAPLESIM && Robot.isSimulation();
    public static RobotContainer instance;

    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double maxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1/4 of a rotation per second
    double speedScalar = 0.7;
    double slownessModifier = 1;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05 * slownessModifier)
            .withRotationalDeadband(maxAngularRate * 0.05 * slownessModifier) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric strafe = new SwerveRequest.RobotCentric()
            .withDeadband(0.075)
            .withRotationalDeadband(0.075)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(maxSpeed);

    public final static CommandXboxController controller1 = new CommandXboxController(0);
    public final static CommandXboxController controller2 = new CommandXboxController(1);
    public static final String vision = null;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralMechanism m_coralMechanism = new CoralMechanism(18);
    public final ElevatorMechanism m_elevatorMechanism = new ElevatorMechanism(13, controller2,
            Constants.kIsAtCompetition);
    public final HangMechanism m_hangMechanism = new HangMechanism(11, controller2);
    // public final HangMechanism m_hangMechanism = new HangMechanism(12,
    // Controller2);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        instance = this;
        NamedCommands.registerCommand("ElevatorL2", new ElevatorMoveToIndex(m_elevatorMechanism, 1));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorMoveToIndex(m_elevatorMechanism, 2));
        NamedCommands.registerCommand("ElevatorCoralStation", new ElevatorMoveToIndex(m_elevatorMechanism, 3));
        NamedCommands.registerCommand("OuttakeCoral", new CoralAutoOuttakeCommand(m_coralMechanism));
        NamedCommands.registerCommand("IntakeCoral", new CoralAutoIntakeCommand(m_coralMechanism));
        NamedCommands.registerCommand("StopCoral", new CoralAutoStopCommand(m_coralMechanism));

        autoChooser = AutoBuilder.buildAutoChooser("Test");
        PathPlannerAuto auto = new PathPlannerAuto("Test");
        // autoChooser.addOption("PhotonAuto", auto);
        SmartDashboard.putData("Auto Mode", autoChooser);

        CameraServer.startAutomaticCapture();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-controller1.getLeftY() * maxSpeed * speedScalar * slownessModifier)
                        .withVelocityY(-controller1.getLeftX() * maxSpeed * speedScalar * slownessModifier)
                        .withRotationalRate(
                                -controller1.getRightX() * maxAngularRate * speedScalar * slownessModifier)));

        // controller1 binding
        controller1.rightBumper().whileTrue(Commands.run(() -> slownessModifier = 0.2));
        controller1.rightBumper().whileFalse(Commands.run(() -> slownessModifier = 1));

        controller1.leftBumper().whileTrue(Commands.run(() -> slownessModifier = 1 / speedScalar));
        controller1.leftBumper().whileFalse(Commands.run(() -> slownessModifier = 1));

        // Controller1.leftTrigger(0.1).whileTrue(drivetrain.applyRequest(() ->
        // strafe.withVelocityY(0.15)));
        // Controller1.rightTrigger(0.1).whileTrue(drivetrain.applyRequest(() ->
        // strafe.withVelocityY(-0.15)));
        controller1.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller1.pov(0).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityX(0.12)));
        controller1.pov(180).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityX(-0.12)));

        controller1.leftTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityY(0.12)));
        controller1.rightTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityY(-0.12)));

        // controller2 binding
        controller2.leftBumper().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 1));
        controller2.rightBumper().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 2));
        controller2.y().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 3));

        // Controller2.pov(0).onTrue(new HangMoveToIndex(m_hangMechanism, 1));
        // Controller2.pov(180).onTrue(new HangMoveToIndex(m_hangMechanism, 0));

        controller2.x().whileTrue(new CoralIntakeCommand(m_coralMechanism));
        controller2.b().whileTrue(new CoralOuttakeCommand(m_coralMechanism));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
