package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import frc.robot.subsystems.CoralMechanism;

import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity
    double speedScalar = 0.7;
    double SlownessModifier = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05 * SlownessModifier).withRotationalDeadband(MaxAngularRate * 0.05 * SlownessModifier) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Controller1 = new CommandXboxController(0);
    private final CommandXboxController Controller2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final SparkMax hangMechanism = new SparkMax(12, MotorType.kBrushless);
    public final CoralMechanism Coral = new CoralMechanism(13, 18);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
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
                drive.withVelocityX(-Controller1.getLeftY() * MaxSpeed * speedScalar * SlownessModifier) // Drive forward with negative Y (forward)
                    .withVelocityY(-Controller1.getLeftX() * MaxSpeed * speedScalar * SlownessModifier) // Drive left with negative X (left)
                    .withRotationalRate(-Controller1.getRightX() * MaxAngularRate * speedScalar * SlownessModifier) // Drive counterclockwise with negative X (left)
            )
        );

        Coral.initDefaultCommand(Controller2);

        Controller2.leftBumper().whileTrue(Commands.parallel(
            Coral.SetIntakeSpeed(0.5),
            drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.15).withVelocityY(0))
        ));
        Controller2.rightBumper().whileTrue(Commands.parallel(
            Coral.SetIntakeSpeed(-0.5),
            drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(-0.15).withVelocityY(0))
        ));

        Controller1.rightBumper().whileTrue(Commands.run(() -> SlownessModifier = 0.35));
        Controller1.rightBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        

        Controller1.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.55).withVelocityY(0))
        );
        Controller1.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Controller1.start().and(Controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Controller1.start().and(Controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        Controller1.back().and(Controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Controller1.back().and(Controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Controller1.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // Controller1.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // reset the field-centric heading on left bumper press
        Controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}