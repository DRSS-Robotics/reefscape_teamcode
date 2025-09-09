
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

public class guh extends SubsystemBase {
//Motor controller, xbox controller, 
    SparkMaxConfig HangConfig = new SparkMaxConfig();

    public final SparkMax hangMotor;
    CommandXboxController joystick;
    public guh(int hangMotorID, CommandXboxController gamepad) {
        hangMotor = new SparkMax(hangMotorID, MotorType.kBrushless);
        joystick = gamepad;

        
        HangConfig.smartCurrentLimit(60);
        HangConfig.voltageCompensation(10);
        HangConfig.apply(ElevCCLoop);
        HangConfig.idleMode(IdleMode.kBrake);

        hangMotor.configure(HangConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        if (joystick.isAButtonPressed()) {
            hangMotor.getClosedLoopController().setReference(Constants.guhHangUp, ControlType.kPosition);
        }else if (joystick.isBButtonPressed()) {
            hangMotor.getClosedLoopController().setReference(Constants.guhHangDown, ControlType.kPosition)
        }
        
    }
}