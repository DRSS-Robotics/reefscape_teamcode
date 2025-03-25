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

public class HangMechanism extends SubsystemBase {

    ClosedLoopConfig HangCCLoop = new ClosedLoopConfig()
            .p(Constants.kElevatorKp).i(Constants.kElevatorKi).d(Constants.kElevatorKd);

    SparkMaxConfig HangConfig = new SparkMaxConfig();

    public final double Deadband = 0.04;
    public final SparkMax HangMotor;
    boolean IsAtTarget = false;
    boolean ShouldGoUp = false;
    public int DesiredCoralHeight;
    CommandXboxController Joystick;
    // This first one is the coral station and the last one is level 3
    // Change the coral station level
    public HangMechanism(int HangID, CommandXboxController Controller) {

        HangMotor = new SparkMax(HangID, MotorType.kBrushless);
        Joystick = Controller;

        HangConfig.voltageCompensation(10);
        HangConfig.smartCurrentLimit(60);
        HangConfig.idleMode(IdleMode.kBrake);
        HangConfig.apply(HangCCLoop);

        HangMotor.configure(HangConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        DriveHang(Joystick);
    }

    public boolean DeadbandCheck(double Value) {
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds(double ControllerY) {
        return !(HangMotor.getEncoder().getPosition() - 8 * ControllerY < Constants.kHangLowerBound ||
                HangMotor.getEncoder().getPosition() - 8 * ControllerY >= Constants.kHangUpperBound);
    }

    public Command DriveHang(CommandXboxController Controller) {
        
        if (DeadbandCheck(Controller.getRightY()) && MotorHeightBounds(Controller.getRightY())) {
            HangMotor.set(-Controller.getRightY());
            return Commands.none();
        } else {
            HangMotor.stopMotor();
            return Commands.none();
        }
    }
}
