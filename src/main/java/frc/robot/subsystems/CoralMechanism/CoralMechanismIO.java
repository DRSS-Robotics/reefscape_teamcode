package frc.robot.subsystems.CoralMechanism;
import edu.wpi.first.epilogue.Logged;


@Logged
public interface CoralMechanismIO {

    @Logged
    public class CoralMechanismIOInputs {
        public boolean hasCoral;
    }

    public void runIntake();

    public void setSpeed(double speed);

    public void update(CoralMechanismIOInputs inputs);
    
}
