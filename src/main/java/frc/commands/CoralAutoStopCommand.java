package frc.commands;

import frc.robot.subsystems.CoralMechanism;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralAutoStopCommand extends Command {
  private final CoralMechanism m_coralMechanism;
  public CoralAutoStopCommand(CoralMechanism cMechanism) {
    m_coralMechanism = cMechanism;
    addRequirements(cMechanism);
  }
  @Override
  public void initialize() {
    m_coralMechanism.runIntake(0.0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
