package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.NamedCommands;

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
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final SparkMax outtakeMotor = new SparkMax(11, MotorType.kBrushed);
    // public TalonFXConfigurator = new TalonFXConfigurator();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final CameraSubsystem m_camera = CameraSubsystem.getInstance();
    
    public RobotContainer() {
        // must register commands and event triggers before building the auto chooser
        NamedCommands.registerCommand("CameraOffset", new CameraCommand());
        NamedCommands.registerCommand("GetCameraData", (Commands.run(() -> {
            drive.withRotationalRate(-0.5 * Math.PI);
            drive.withVelocityY(0.0762);
        })));
        
        // new EventTrigger("GetCameraData").onTrue(Commands.sequence(Commands.runOnce(() -> 
        // {
        //     drive.withRotationalRate(-0.25 * Math.PI);
        //     drive.withVelocityX(0.5);
        // }
        // ), Commands.waitSeconds(5),Commands.runOnce (() -> 
        // {
        // CommandScheduler.getInstance().enable();
        // }
        // ),Commands.print("Commands ran")));

      

//comand class try
        autoChooser = AutoBuilder.buildAutoChooser("CalibAutoTest");
        SmartDashboard.putData("Auto Mode", autoChooser);
    
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedScalar * SlownessModifier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedScalar * SlownessModifier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedScalar * SlownessModifier) // Drive counterclockwise with negative X (left)
            )
        );

        // set these to joystick2 later
        // We fixed it for you- Micah and William L.
        joystick2.a().whileTrue(Commands.run(() -> {
            outtakeMotor.set(0.5);
        }));
        joystick2.a().whileFalse(Commands.run(() -> {
            outtakeMotor.set(0.0);
        }));
        joystick2.x().whileTrue(Commands.run(() -> {
            outtakeMotor.set(-0.3);
        }));
        joystick2.x().whileFalse(Commands.run(() -> {
            outtakeMotor.set(0);
        }));
        joystick2.y().whileTrue(Commands.run(() -> {
            outtakeMotor.set(-0.15);
        }));
        joystick2.y().whileFalse(Commands.run(() -> {
            outtakeMotor.set(0);
        }));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-0.25 * Math.PI) // clockwise
            )
        );
        // joystick.x().whileTrue(drivetrain.applyRequest(() ->
        //     drive.withVelocityX(-0.5) // forward
        //         .withVelocityY(0)
        //         .withRotationalRate(0)
        //     // point.withAngle(Rotation2d.fromDegrees(0));
        //     )
        joystick.rightBumper().whileTrue(Commands.run(() -> SlownessModifier = 0.35));
        joystick.rightBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.55).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }


    } 