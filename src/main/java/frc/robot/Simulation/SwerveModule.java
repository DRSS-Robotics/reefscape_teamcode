// package frc.robot.Simulation;

// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;

// public class SwerveModule {
//     private final ModuleIO io;
//     private final String name;

//     private SwerveModuleState setPoint;
//     private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

//     public SwerveModule(ModuleIO io, String Name) {
//         this.io = io;
//         this.name = name;

//         //may not be needed
//         // this.driveMotorHardwareFault.set(false);
//         // this.steerMotorHardwareFault.set(false);
//         // this.steerEncoderHardwareFault.set(false);

//         setPoint = new SwerveModuleState();
//         io.setDriveBreak(true);
//         io.setSteerBreak(true);
//     }
// }
