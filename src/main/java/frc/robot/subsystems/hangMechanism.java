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

    ClosedLoopConfig hangCCLoop = new ClosedLoopConfig()
            .p(Constants.kElevatorKp).i(Constants.kElevatorKi).d(Constants.kElevatorKd);

    SparkMaxConfig hangConfig = new SparkMaxConfig();

    public final double deadband = 0.04;
    public final SparkMax hangMotor;
    boolean isAtTarget = false;
    boolean shouldGoUp = false;
    public int desiredCoralHeight;
    CommandXboxController joystick;
    // This first one is the coral station and the last one is level 3
    // Change the coral station level
    public HangMechanism(int HangID, CommandXboxController Controller) {

        hangMotor = new SparkMax(HangID, MotorType.kBrushless);
        joystick = Controller;

        hangConfig.voltageCompensation(10);
        hangConfig.smartCurrentLimit(60);
        hangConfig.idleMode(IdleMode.kBrake);
        hangConfig.apply(hangCCLoop);

        hangMotor.configure(hangConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        driveHang(joystick);
    }

    public boolean deadbandCheck(double Value) {
        return Math.abs(Value) > deadband;
    }

    public boolean motorHeightBounds(double controllerY) {
        return !(hangMotor.getEncoder().getPosition() - 8 * controllerY < Constants.kHangLowerBound ||
                hangMotor.getEncoder().getPosition() - 8 * controllerY >= Constants.kHangUpperBound);
    }

    public Command driveHang(CommandXboxController controller) {
        if (deadbandCheck(controller.getRightY()) && motorHeightBounds(controller.getRightY())) {
            hangMotor.set(-controller.getRightY());
        } else {
            hangMotor.stopMotor();
        }
        return Commands.none();
    }
}
