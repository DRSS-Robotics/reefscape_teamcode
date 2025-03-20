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
import frc.commands.coralIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.coralMechanism;
import frc.robot.subsystems.elevatorMechanism;
import frc.robot.subsystems.hangMechanism;
import frc.commands.coralIntakeCommand;
import frc.commands.coralOuttakeCommand;
import frc.commands.CoralMoveToIndex;
import frc.commands.CoralSetDesiredHeight;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity
    double speedScalar = 0.7;
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

    private final CommandXboxController Controller1 = new CommandXboxController(0);
    public final static CommandXboxController Controller2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final coralMechanism coralMechanism = new coralMechanism();
    public final hangMechanism hangMechanism = new hangMechanism();
    public final elevatorMechanism elevatorMechanism = new elevatorMechanism();

    // public TalonFXConfigurator = new TalonFXConfigurator();

    /* Path follower */ 
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // drivetrain.getModules()[0].getDriveMotor()
        //must register commands and event triggers before building the auto chooser
        //new EventTrigger("test-OneThird").onTrue(Commands.sequence(Commands.runOnce(() -> {CommandScheduler.getInstance().disable();}),Commands.waitSeconds(5),Commands.runOnce(() -> {CommandScheduler.getInstance().enable();}),Commands.print("yes")));

        NamedCommands.registerCommand("LiftElevatorLevel2", new CoralMoveToIndex(elevatorMechanism, 1));
        NamedCommands.registerCommand("LiftElevatorLevel3", new CoralMoveToIndex(elevatorMechanism, 2));
        //NamedCommands.registerCommand("LowerElevator", Commands.run(()  -> elevatorMechanism.set(-0.75)));
        //NamedCommands.registerCommand("StopElveator", Commands.run(()  -> elevatorMechanism.set(0.0)));
        NamedCommands.registerCommand("DropCoral", new coralOuttakeCommand(coralMechanism));
        NamedCommands.registerCommand("pickupCoral", new coralIntakeCommand(coralMechanism));

        //Don't use
        // NamedCommands.registerCommand("dropCoral", Commands.run(()  -> coralIntake.set(0.75)));
        // NamedCommands.registerCommand("pickupAlgae", new coralOuttakeCommand(coralMechanism));
        // NamedCommands.registerCommand("dropAlgae", Commands.run(()  -> coralIntake.set(-0.75)));


        // NamedCommands.registerCommand("StopCoralIntake", Commands.run(()  ->  coralIntake.set(0.0)));
        autoChooser = AutoBuilder.buildAutoChooser("Straight");
        // PathPlannerAuto auto = new PathPlannerAuto("L2Middle");
        
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


        // l2 is 15.0 on enc readings
        // l3 is 110.0

         // Controller2.y().whileTrue();
        // Controller2.y().whileFalse();
        Controller2.x().whileTrue(new coralOuttakeCommand(coralMechanism));
        Controller2.x().whileFalse(new coralOuttakeCommand(coralMechanism));
        Controller2.b().whileTrue(new coralIntakeCommand(coralMechanism));
        Controller2.b().whileFalse(new coralIntakeCommand(coralMechanism));
      
        // Controller2.leftBumper().whileTrue(Commands.run(() -> {

        //     if (elevatorMechanism.getEncoder().getPosition() < 110.0) {
        //         elevatorMechanism.set(0.65);
        //     }
        // })); 
        // Controller2.leftBumper().whileFalse(Commands.run(() -> {
        //     System.out.println(elevatorMechanism.getEncoder().getPosition());
        //     elevatorMechanism.set(0);
        // }));
        // Controller2.rightBumper().whileTrue(Commands.run(() -> {
        //     if(elevatorMechanism.getEncoder().getPosition() > 3.0 || Controller2.start().getAsBoolean()) {
        //         elevatorMechanism.set(-0.65);
        //     }
        // }));
        // Controller2.rightBumper().whileFalse(Commands.run(() -> {
        //     System.out.println(elevatorMechanism.getEncoder().getPosition());
        //     elevatorMechanism.set(0);
        // }));


        // joystick.x().whileTrue(drivetrain.applyRequest(() ->
        //     drive.withVelocityX(-0.5) // forward
        //         .withVelocityY(0)
        //         .withRotationalRate(0)
        //     // point.withAngle(Rotation2d.fromDegrees(0));
        //     )

        Controller1.rightBumper().whileTrue(Commands.run(() -> SlownessModifier = 0.3));
        Controller1.rightBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        Controller1.leftBumper().whileTrue(Commands.run(() -> SlownessModifier = 1 / speedScalar));
        Controller1.leftBumper().whileFalse(Commands.run(() -> SlownessModifier = 1));

        // Nathan was here
        // i hate you nathan -felix

        // reset the field-centric heading on left bumper press
        // Controller1.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        Controller1.b().onTrue(Commands.runOnce(() ->  System.out.println(elevatorMechanism.elevatorMechanism.getEncoder().getPosition())));
        Controller1.a().onTrue(Commands.runOnce(() ->  elevatorMechanism.elevatorMechanism.getEncoder().setPosition(0.0)));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
