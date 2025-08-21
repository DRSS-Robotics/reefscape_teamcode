package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism.CoralMechanism;

public class CoralStopCommand extends Command {
  private final CoralMechanism m_coralMechanism;
  public CoralStopCommand(CoralMechanism cMechanism) {
    m_coralMechanism = cMechanism;
    addRequirements(cMechanism);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_coralMechanism.runIntake(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_coralMechanism.runIntake(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
