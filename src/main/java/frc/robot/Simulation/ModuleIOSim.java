// package frc.robot.Simulation;

// import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
// import org.ironmaple.simulation.motorsims.SimulatedMotorController;
// import frc.robot.generated.TunerConstants;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Voltage;

// public class ModuleIOSim implements ModuleIO {
//     private final SwerveModuleSimulation moduleSimulation;
//     private final SimulatedMotorController.GenericMotorController driveController;
//     private final SimulatedMotorController.GenericMotorController turnController;

//     public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
//         this.moduleSimulation = moduleSimulation;

//         this.driveController = moduleSimulation
//         .useGenericMotorControllerForDrive()
//         .withCurrentLimits(Amps.of(60));
        
//         this.turnController = moduleSimulation
//         .useGenericControllerForSteer()
//         .withCurrentLimit(Amps.of(20));
//     }

//     public void setDriveVoltage (Voltage voltage){
//         this.driveController.requestVoltage(voltage);
//     }

//     public void setTurnVoltage (Voltage voltage){
//         this.turnController.requestVoltage(voltage);
//     }

//     public Rotation2d getSteerFacing(){
//         return this.moduleSimulation.getSteerAbsoluteFacing();
//     }

//     public Angle getSteerRelativePosition(){
//         return this.moduleSimulation.getSteerRelativeEncoderPosition().divide(TunerConstants.kSteerGearRatio);
//     }

//     public Angle getDriveWheelPosition(){
//         return this.moduleSimulation.getDriveWheelFinalPosition();
//     }
// }
