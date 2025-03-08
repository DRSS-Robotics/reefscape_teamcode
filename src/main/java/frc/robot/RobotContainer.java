package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralMechanism;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity
    double speedScalar = 0.35;
    double SlownessModifier = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05 * SlownessModifier).withRotationalDeadband(MaxAngularRate * 0.05 * SlownessModifier) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralMechanism Coral = new CoralMechanism(11, 12);


    // public TalonFXConfigurator = new TalonFXConfigurator();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // drivetrain.getModules()[0].getDriveMotor()
        //must register commands and event triggers before building the auto chooser
        //new EventTrigger("test-OneThird").onTrue(Commands.sequence(Commands.runOnce(() -> {CommandScheduler.getInstance().disable();}),Commands.waitSeconds(5),Commands.runOnce(() -> {CommandScheduler.getInstance().enable();}),Commands.print("yes")));

        autoChooser = AutoBuilder.buildAutoChooser("CalibAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller1.getLeftY() * MaxSpeed * speedScalar * SlownessModifier) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller1.getLeftX() * MaxSpeed * speedScalar * SlownessModifier) // Drive left with negative X (left)
                    .withRotationalRate(-controller1.getRightX() * MaxAngularRate * speedScalar * SlownessModifier) // Drive counterclockwise with negative X (left)
            )
        );

        controller1.rightBumper().whileTrue(Commands.run(() -> SlownessModifier = 0.35));
        controller1.rightBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        controller2.leftBumper().whileTrue(Commands.parallel(
            Coral.SetIntakeSpeed(0.5),
            drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.15).withVelocityY(0))
        ));
        controller2.rightBumper().whileTrue(Commands.parallel(
            Coral.SetIntakeSpeed(-0.5),
            drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(-0.15).withVelocityY(0))
        ));

        Command CoralElevatorCommand = Coral.DriveElevator(controller2).repeatedly();
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // controller1.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // controller1.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}