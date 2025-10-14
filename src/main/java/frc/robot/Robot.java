// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Simulation.MapleSimDriveTrain;
import frc.robot.Simulation.SimLogic;
import frc.robot.Simulation.SimulatedAIRobot;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;



public class Robot extends TimedRobot {
  XboxController controller = new XboxController(0);
   private CameraSubsystem cam = new CameraSubsystem();
  private Optional<EstimatedRobotPose> poseEstimate;

  // private RobotContainer robotContainer = new RobotContainer();
  private Command m_autonomousCommand;
  private List<SimulatedAIRobot> simulatedAIRobots = new ArrayList<>();

  private final RobotContainer m_robotContainer;
  private static boolean isBlueAlliance = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    //starting to log joystick movments and the driver station
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    CommandScheduler.getInstance().run();
  //   poseEstimate = cam.getPoseEstimate();
  //  if (poseEstimate.isPresent()) {
  //     EstimatedRobotPose visionPose = poseEstimate.get();
  //     System.out.println(visionPose.estimatedPose);
  //     m_robotContainer.drivetrain.addVisionMeasurement(visionPose.estimatedPose.toPose2d(),
  //         visionPose.timestampSeconds);
  //   } else {
  //      System.out.println("No pose estimate");
  // }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println(m_robotContainer.m_hangMechanism.HangMotor.getEncoder().getPosition());

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  StructArrayPublisher<Pose2d> aiRobotPoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("AI Robot Poses", Pose2d.struct).publish();

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Coral Poses", Pose3d.struct).publish();

  @Override
  public void simulationPeriodic() {
    RobotContainer rc = m_robotContainer;
    Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    coralPoses.accept(coral);
    //  Pose3d robotPose = new Pose3d(rc.drivetrain.getSimPose());

      Pose2d[] airobotPosesArray = new Pose2d[simulatedAIRobots.size()];
      for (int i = 0; i < simulatedAIRobots.size(); i++) {
        airobotPosesArray[i] = simulatedAIRobots.get(i).getPose();
      }
      aiRobotPoses.accept(airobotPosesArray);
      if (RobotContainer.MAPLESIM) {
        // appears to be necessary based on MapleSim, but had side effects (increased sensitivity in motion... SPINNING)
        //SimulatedArena.getInstance().simulationPeriodic();
      }
  }

  @Override
  public void simulationInit() {
    if (RobotContainer.MAPLESIM) {
      SimLogic.spawnHumanPlayerCoral(true);
      SimLogic.spawnHumanPlayerCoral(false);
      simulatedAIRobots.add(new SimulatedAIRobot(0));
    }
  }

  public static boolean isBlue(){
    return isBlueAlliance;
  }
  public static boolean isRed(){
    return !isBlue();
  }
}
