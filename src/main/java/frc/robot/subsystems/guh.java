
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

    boolean jimGuh;

    ClosedLoopConfig HangCCLoop = new ClosedLoopConfig()
            .p(Constants.kHangKp).i(Constants.kHangKi).d(Constants.kHangKd);

    public final SparkMax hangMotor;
    CommandXboxController joystick;

    //Constructor guh
    public guh(int hangMotorID, CommandXboxController gamepad) {
        hangMotor = new SparkMax(hangMotorID, MotorType.kBrushless);
        joystick = gamepad;
        jimGuh = true;
        
        HangConfig.smartCurrentLimit(60);
        HangConfig.voltageCompensation(10);
        HangConfig.apply(HangCCLoop);
        HangConfig.idleMode(IdleMode.kBrake);

        hangMotor.configure(HangConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // if (joystick.a().getAsBoolean()) {
        //     hangMotor.getClosedLoopController().setReference(Constants.guhHangUp, ControlType.kPosition);
            
        //     if (jimGuh) {
        //         System.out.println("Guh hello");
        //     }
        //     jimGuh = false;
        // }else if (joystick.b().getAsBoolean()) {
        //     hangMotor.getClosedLoopController().setReference(Constants.guhHangDown, ControlType.kPosition);
        // }else if (!joystick.a().getAsBoolean()) {
        //     jimGuh = true;
        // }

        if (!jimGuh && (joystick.a().getAsBoolean() || joystick.b().getAsBoolean())) {
            System.out.println("Guh hello");
        }

        if (joystick.a().getAsBoolean()) {
            hangMotor.getClosedLoopController().setReference(Constants.guhHangUp, ControlType.kPosition);

        }else if (joystick.b().getAsBoolean()) {
            hangMotor.getClosedLoopController().setReference(Constants.guhHangDown, ControlType.kPosition);
        }

        jimGuh = joystick.a().getAsBoolean() || joystick.b().getAsBoolean();
    }
}