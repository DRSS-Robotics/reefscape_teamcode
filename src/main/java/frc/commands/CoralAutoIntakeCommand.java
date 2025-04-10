
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import frc.robot.subsystems.CoralMechanism;
import edu.wpi.first.wpilibj2.command.Command;

/** An ArmUpCommand that uses an Arm subsystem. */
public class CoralAutoIntakeCommand extends Command {
  private final CoralMechanism m_coralMechanism;

  /**
   * Powers the arm up, when finished passively holds the arm up.
   * 
   * We recommend that you use this to only move the arm into the hardstop
   * and let the passive portion hold the arm up.
   *
   * @param arm The subsystem used by this command.
   */
  public CoralAutoIntakeCommand(CoralMechanism cMechanism) {
    m_coralMechanism = cMechanism;
    addRequirements(cMechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_coralMechanism.runIntake(-0.5);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
