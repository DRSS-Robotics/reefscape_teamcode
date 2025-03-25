
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ElevatorMechanism extends SubsystemBase {

    ClosedLoopConfig ElevCCLoop = new ClosedLoopConfig()
            .p(Constants.kElevatorKp).i(Constants.kElevatorKi).d(Constants.kElevatorKd);

    SparkMaxConfig ElevConfig = new SparkMaxConfig();

    public final double Deadband = 0.04;
    public final SparkMax elevatorMotor;
    boolean IsAtTarget = false;
    boolean ShouldGoUp = false;
    public int DesiredCoralHeight;
    CommandXboxController Joystick;
    // This first one is the coral station and the last one is level 3
    // Change the coral station level

    boolean PrevElevDir = true;

    /**
     * This subsytem that controls the arm.
     */
    public ElevatorMechanism(int ElevID, CommandXboxController Controller) {

        elevatorMotor = new SparkMax(ElevID, MotorType.kBrushless);
        Joystick = Controller;

        ElevConfig.voltageCompensation(10);
        ElevConfig.smartCurrentLimit(60);
        ElevConfig.idleMode(IdleMode.kBrake);
        ElevConfig.apply(ElevCCLoop);

        elevatorMotor.configure(ElevConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        DriveElevator(Joystick);
    }

    public boolean DeadbandCheck(double Value) {
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds(double ControllerY) {
        if (elevatorMotor.getEncoder().getPosition() - 8 * ControllerY < -3 ||
                elevatorMotor.getEncoder().getPosition() - 8 * ControllerY >= 120) {
            return false;
        } else {
            return true;
        }
    }

    public Command DriveElevator(CommandXboxController Controller) {
        
        if (DeadbandCheck(Controller.getRightY()) && MotorHeightBounds(Controller.getRightY())) {
            elevatorMotor.set(-Controller.getRightY());
            return Commands.none();
        } else {
            elevatorMotor.stopMotor();
            return Commands.none();
        }
    }

    public Command SetDesiredElevatorHeight(int CoralLevelIndex) {
        IsAtTarget = false;
        DesiredCoralHeight = CoralLevelIndex;
        return Commands.none();
    }

    public Command DriveToPosition(double InPos) {
        // ShouldGoUp = (Elevator.getEncoder().getPosition() <
        // Constants.kElevatorTargetHeights[DesiredCoralHeight]);
        elevatorMotor.getClosedLoopController().setReference(InPos, ControlType.kPosition);
        return Commands.none();
    }

}
