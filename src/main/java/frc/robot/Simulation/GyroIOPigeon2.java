// package frc.robot.Simulation;

// import java.util.Queue;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.Odometry;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import frc.robot.Simulation.GyroIO;
// import frc.robot.generated.TunerConstants;

// public class GyroIOPigeon2 implements GyroIO {
//     private final Pigeon2 pigeon;
//     private final StatusSignal<Angle> yaw;
//     private final Queue<Angle> yawPosition;
//     private final StatusSignal<AngularVelocity> yawAngularVelocity;

//     private final boolean ConfigurationOk;

//    // public GyroIOPigeon2(SwerveModuleConstants)

//   //  public GyroIOPigeon2(){
//         pigeon = new Pigeon2(0);
//         //Check id, this might not be it
//         yaw = pigeon.getYaw();
//         yawAngularVelocity = pigeon.getAngularVelocityZWorld();
//         yawPosition = OdometryThread.registerSignalSignal(yaw);

//         BaseStatusSignal.setUpdateFrequencyForAll(100.0, yaw);
//         BaseStatusSignal.setUpdateFrequencyForAll(TunerConstants.ODOEMTRY_FREQUENCY, yaw);
//         pigeon.optimizeBusUtilization();
//     }

//     @Override
//     public void updateInputs (GyroIOInputs Inputs) {
//     inputs.configurationFailed = !ConfigurationOK;

//     }
     
//     // public Rotation2d getGyroRotation(){
//     //     return pigeon.getRotation2d();
//     // }

    
//     // public AngularVelocity getGyroAngularVelocity(){
//     //     return pigeon.get
//     // }
// }
