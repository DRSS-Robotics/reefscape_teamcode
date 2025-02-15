// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae_Mechanism;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.Algae_Mechanism_Down_Command;
import frc.robot.commands.Algae_Mechanism_Intake_Command;
import frc.robot.commands.Algae_Mechanism_Outake_Command;
import frc.robot.commands.Algae_Mechanism_Up_Command;
import edu.wpi.first.wpilibj.Encoder;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxSpeedScalar = 0.20;
    private Algae_Mechanism algaeMechanism = new Algae_Mechanism();
    private Algae_Mechanism_Intake_Command algaeIntakeCommand = new Algae_Mechanism_Intake_Command(algaeMechanism);
    private Algae_Mechanism_Outake_Command algaeOuttakeCommand = new Algae_Mechanism_Outake_Command(algaeMechanism);
    private Algae_Mechanism_Up_Command algaeUpCommand = new Algae_Mechanism_Up_Command(algaeMechanism);
    private Algae_Mechanism_Down_Command algaeDownCommand = new Algae_Mechanism_Down_Command(algaeMechanism);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    public final XboxController gamepad2 = new XboxController(0);
    public static PWMMotorController spinnerWheelMotorController = new PWMMotorController("spinnerWheelMotorController",0) {};
    public static PWMMotorController pivotMotorController = new PWMMotorController("pivotMotorController",0) {};
    public static double speedOfSpinnerWheels = 0.5;
    public static double speedOfPivotMotor = 0.5;

    public static DCMotor spinnerWheelMotor = new DCMotor(12,0.97,100,1.4,1151.9,1);
    public static DCMotor pivotMotor = new DCMotor(12,0.97,100,1.4,1151.9,1);
    public static Encoder pivotEncoder = new Encoder(0,0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed * MaxSpeedScalar) // Drive forward with Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed * MaxSpeedScalar) // Drive left with X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * MaxSpeedScalar) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.-
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Trigger rightBumper = new JoystickButton(gamepad2, XboxController.Button.kRightBumper.value);
        Trigger leftBumper = new JoystickButton(gamepad2, XboxController.Button.kLeftBumper.value);
        Trigger buttonX = new JoystickButton(gamepad2, XboxController.Button.kX.value);
       // Trigger isUp = new algaeMechanism.isUp(); 

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        //When the right bumper is pressed, execute the algae mechanism command
        rightBumper.onTrue(algaeMechanism.Algae_Intake_Command(spinnerWheelMotor, spinnerWheelMotorController, speedOfSpinnerWheels));
        leftBumper.onTrue(algaeMechanism.Algae_Outake_Command(spinnerWheelMotor, spinnerWheelMotorController, speedOfSpinnerWheels));
    
        if (!algaeMechanism.isUp(null)){
            buttonX.onTrue(algaeMechanism.Algae_Up_Command(pivotMotor, pivotMotorController, speedOfPivotMotor));
        } else if (algaeMechanism.isUp(null)){
            buttonX.onTrue(algaeMechanism.Algae_Down_Command(pivotMotor, pivotMotorController, speedOfPivotMotor));
        } else{

        }
        }
        

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    //test
}
