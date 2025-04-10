// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

  public static final boolean kIsAtCompetition = true;

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final double kElevatorKp = 0.15;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 0;
  // ignore index 0; index 1 is l2; index 2 is l3; index 3 is coral station
  public static final double[] kPracticeElevatorTargetHeights = {20, 15.0, 110.0, 42.0} /* testing at school */;
  public static final double[] kCompElevatorTargetHeights = {20, 14.2, 112.0, 42.0} /* real comp values */;
  // bounds only used with joystick control, ignored by teleop auto
  public static final double kElevatorLowerBound = -0.5;
  public static final double kElevatorUpperBound = 123.0;

  public static final double kHangKp = 0.15;
  public static final double kHangKi = 0;
  public static final double kHangKd = 0;
  public static final double[] kHangTargetRotations = {0.0, 90.0};
  public static final double kHangLowerBound = -0.5;
  public static final double kHangUpperBound = 3500.0;

  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kElevatorGearing = 10.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double kCarriageMass = 4.0; // kg

  public static final double kSetpointMeters = 0.75;
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = 0.0;
  public static final double kMaxElevatorHeightMeters = 1.25;

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  public static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;
}
