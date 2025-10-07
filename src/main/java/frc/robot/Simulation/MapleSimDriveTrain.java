package frc.robot.Simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.rmi.Remote;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import frc.robot.Simulation.*;
import frc.robot.generated.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
//import com.ctre.phoenix6.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.measure.*;

public class MapleSimDriveTrain {
    private final Pigeon2SimState pigeonSim;
    public final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    public MapleSimDriveTrain(
        Time simPeriod,
        Mass robotMassWithBumpers,
        Distance bumperLengthX,
        Distance bumperWidthY,
        DCMotor driveMotorModel,
        DCMotor steerMotorModel,
        double wheelCOF,
        Translation2d[] moduleLocations,
        Pigeon2 pigeon,
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
        moduleConstants) {
            this.pigeonSim = pigeon.getSimState();
            simModules = new SimSwerveModule[moduleConstants.length];
            DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(new SwerveModuleSimulationConfig(
            driveMotorModel,
            steerMotorModel,
            moduleConstants[0].DriveMotorGearRatio,
            moduleConstants[0].SteerMotorGearRatio,
            Volts.of(moduleConstants[0].DriveFrictionVoltage),
            Volts.of(moduleConstants[0].SteerFrictionVoltage),
            Meters.of(moduleConstants[0].WheelRadius),
            KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
            wheelCOF));

            mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());
            SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
            for (int i = 0; i < this.simModules.length; i++)
            simModules[i] = new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);

            SimulatedArena.overrideSimulationTimings(simPeriod, 1);
            SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }
    public SwerveDriveSimulation getDriveSim() {
        if (mapleSimDrive != null) {
            return mapleSimDrive;
        }
        return null;
    }

    // public void update() {
    //     System.out.println("Updating");
    //     SimulatedArena.getInstance().simulationPeriodic();
    //     pigeonSim.setRawYaw(
    //         mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    //     pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
    //         mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
    // }

    protected static class SimSwerveModule {
        public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants;
        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule (
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants,
            SwerveModuleSimulation moduleSimulation,
            SwerveModule<TalonFX, TalonFX, CANcoder> module) {
                this.moduleConstants = moduleConstants;
                this.moduleSimulation = moduleSimulation;
                moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(module.getDriveMotor()));
                moduleSimulation.useSteerMotorController(new TalonFXControllerWithRemoteCanCoderSim(module.getSteerMotor(), module.getEncoder()));
            }
        }

        public static class TalonFXMotorControllerSim implements SimulatedMotorController{
            public final int id;
            private final TalonFXSimState talonFXSimState;

            public TalonFXMotorControllerSim(TalonFX talonFX){
                this.id = talonFX.getDeviceID();
                this.talonFXSimState = talonFX.getSimState();
            }

            @Override
            public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity angularVelocity, Angle encoderAngle, AngularVelocity encoderVelocity){
                talonFXSimState.setRawRotorPosition(encoderAngle);
                talonFXSimState.setRotorVelocity(encoderVelocity);
                talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

                return talonFXSimState.getMotorVoltageMeasure();
            }
        }

            public static class TalonFXControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
                private final int encoderId;
                private final CANcoderSimState remoteCancoderSimState;
                
                public TalonFXControllerWithRemoteCanCoderSim(TalonFX TalonFX, CANcoder cancoder) {
                    super(TalonFX);
                    this.remoteCancoderSimState = cancoder.getSimState();
                    this.encoderId = cancoder.getDeviceID();
                }

        
            @Override
            public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity) {
                
                remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
                remoteCancoderSimState.setRawPosition(mechanismAngle);
                remoteCancoderSimState.setVelocity(mechanismVelocity);

                return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
            }

            public static SwerveModuleConstants<?,?,?>[] regulateModuleConstantsForSimulation(
                SwerveModuleConstants<?,?,?>[] moduleConstants
            ){
                for(SwerveModuleConstants<?,?,?> moduleConstant : moduleConstants){
                    regulateModuleConstantsForSimulation(moduleConstants);
                }
                return moduleConstants;

            }

            public static void regulateModuleConstantsForSimulation(SwerveModuleConstants<?,?,?> moduleConstants) {
                if 
                (RobotBase.isReal()) return;

                moduleConstants

                .withEncoderOffset(0)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)

                .withSteerMotorGains(new Slot0Configs()
                    .withKP(16).withKI(0.0).withKD(0.4)
                    .withKS(0.1).withKV(1.43).withKA(0.039)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(12.1)
                .withSteerFrictionVoltage(Volts.of(0.2))
                .withDriveFrictionVoltage(Volts.of(0.2))
                .withSteerInertia(KilogramSquareMeters.of(0.01));
            }
        }
        
        
    }
    
