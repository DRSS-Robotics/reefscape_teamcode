package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HangMechanism extends SubsystemBase {

    public final double Deadband = 0.04;
    private final SparkMax HangMotor;
    private CommandXboxController Joystick;

    public HangMechanism(int HangID, CommandXboxController Controller) {

        HangMotor = new SparkMax(HangID, MotorType.kBrushless);
        Joystick = Controller;
        SparkMaxConfig hangConfig = new SparkMaxConfig();
        hangConfig.voltageCompensation(10);
        hangConfig.smartCurrentLimit(60);
        hangConfig.idleMode(IdleMode.kBrake);
        HangMotor.configure(hangConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        DriveHang(Joystick);
    }

    public boolean DeadbandCheck(double Value) {
        // System.out.println(Math.abs(Value) > Deadband);
        return Math.abs(Value) > Deadband;
    }

    public boolean MotorHeightBounds(double ControllerY) {

        if (HangMotor.getEncoder().getPosition() - 8 * ControllerY < -3 ||
                HangMotor.getEncoder().getPosition() - 8 * ControllerY >= -120) {
            return false;
        } else {
            return true;
        }
    }

    public Command DriveHang(CommandXboxController Controller) {
        System.out.println(HangMotor.getEncoder().getPosition());
        if (DeadbandCheck(Controller.getRightY()) /*&& MotorHeightBounds(Controller.getRightY())*/) {
            HangMotor.set(-Controller.getRightY());
            return Commands.none();
        } else {
            HangMotor.stopMotor();
            return Commands.none();
        }
    }

}
