
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import frc.robot.subsystems.CoralMechanism;
import edu.wpi.first.wpilibj2.command.Command;

/** An ArmUpCommand that uses an Arm subsystem. */
public class CoralOuttakeCommand extends Command {
  private final CoralMechanism m_coralMechanism;

  /**
   * Powers the arm up, when finished passively holds the arm up.
   * 
   * We recommend that you use this to only move the arm into the hardstop
   * and let the passive portion hold the arm up.
   *
   * @param arm The subsystem used by this command.
   */
  public CoralOuttakeCommand(CoralMechanism cMechanism) {
    m_coralMechanism = cMechanism;
    addRequirements(cMechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coralMechanism.runIntake(0.5);
  }

  // Called once the command ends or is interrupted.
  // Here we run a command that will hold the arm up after to ensure the arm does
  // not drop due to gravity.
  @Override
  public void end(boolean interrupted) {
    m_coralMechanism.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
