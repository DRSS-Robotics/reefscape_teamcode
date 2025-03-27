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
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.ElevatorMechanism;
import frc.robot.subsystems.HangMechanism;

import frc.commands.CoralIntakeCommand;
import frc.commands.CoralAutoIntakeCommand;
import frc.commands.CoralOuttakeCommand;
import frc.commands.CoralAutoOuttakeCommand;
import frc.commands.CoralStopCommand;
import frc.commands.ElevatorMoveToIndex;
import frc.commands.HangMoveToIndex;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1/4 of a rotation per second
    double speedScalar = 0.7;
    double SlownessModifier = 1;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05 * SlownessModifier)
            .withRotationalDeadband(MaxAngularRate * 0.05 * SlownessModifier) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric strafe = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05 * SlownessModifier)
            .withRotationalDeadband(MaxAngularRate * 0.05 * SlownessModifier) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Controller1 = new CommandXboxController(0);
    public final static CommandXboxController Controller2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final CoralMechanism m_coralMechanism = new CoralMechanism(18);
    public final ElevatorMechanism m_elevatorMechanism = new ElevatorMechanism(13, Controller2, Constants.kIsAtCompetition);
    // public final HangMechanism m_hangMechanism = new HangMechanism(12, Controller2);

    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {

        NamedCommands.registerCommand("ElevatorL2", new ElevatorMoveToIndex(m_elevatorMechanism, 1));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorMoveToIndex(m_elevatorMechanism, 2));
        NamedCommands.registerCommand("ElevatorCoralStation", new ElevatorMoveToIndex(m_elevatorMechanism, 3));
        NamedCommands.registerCommand("OuttakeCoral", new CoralAutoOuttakeCommand(m_coralMechanism));
        NamedCommands.registerCommand("IntakeCoral", new CoralAutoIntakeCommand(m_coralMechanism));
        NamedCommands.registerCommand("StopCoral", new CoralStopCommand(m_coralMechanism));
        
        autoChooser = AutoBuilder.buildAutoChooser("TwoCoralScore_Path3");
        // PathPlannerAuto auto = new PathPlannerAuto("L2Middle");

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
                        .withVelocityX(-Controller1.getLeftY() * MaxSpeed * speedScalar * SlownessModifier)
                        .withVelocityY(-Controller1.getLeftX() * MaxSpeed * speedScalar * SlownessModifier)
                        .withRotationalRate(
                                -Controller1.getRightX() * MaxAngularRate * speedScalar * SlownessModifier)));
        
        // controller1 binding
        Controller1.rightBumper().whileTrue(Commands.run(() -> SlownessModifier = 0.2));
        Controller1.rightBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        Controller1.leftBumper().whileTrue(Commands.run(() -> SlownessModifier = 1 / speedScalar));
        Controller1.leftBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        Controller1.leftTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityY(0.15)));
        Controller1.rightTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> strafe.withVelocityY(-0.15)));
        Controller1.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // controller2 binding
        Controller2.leftBumper().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 1));
        Controller2.rightBumper().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 2));
        Controller2.y().onTrue(new ElevatorMoveToIndex(m_elevatorMechanism, 3));

        // Controller2.pov(0).onTrue(new HangMoveToIndex(m_hangMechanism, 1));
        // Controller2.pov(180).onTrue(new HangMoveToIndex(m_hangMechanism, 0));

        Controller2.x().whileTrue(new CoralOuttakeCommand(m_coralMechanism));
        Controller2.b().whileTrue(new CoralIntakeCommand(m_coralMechanism));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
