// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CameraSubsystem;

public class Robot extends TimedRobot {
  XboxController controller = new XboxController(0);
  // private CameraSubsystem cam = new CameraSubsystem();
  private Optional<EstimatedRobotPose> poseEstimate;
  // private RobotContainer robotContainer = new RobotContainer();
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.updateDashboardValues();
    CommandScheduler.getInstance().run();
    // poseEstimate = cam.getPoseEstimate();
    // // if (poseEstimate.isPresent()) {
    // if (false) {
    //   EstimatedRobotPose visi+onPose = poseEstimate.get();
    //   System.out.println(visionPose.estimatedPose);
    //   m_robotContainer.drivetrain.addVisionMeasurement(visionPose.estimatedPose.toPose2d(),
    //       visionPose.timestampSeconds);
    // } else {
    //   // System.out.println("No pose estimate");
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

  @Override
  public void simulationPeriodic() {
  }
}
