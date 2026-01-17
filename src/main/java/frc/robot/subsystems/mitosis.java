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

public class mitosis extends SubsystemBase {
    private final SparkMax moss;

    public final double deadPan = 0.04;
    CommandXboxController mossCont;

    public boolean IsAtComp;

    public ElevatorMechanism(int ElevID, CommandXboxController Controller, boolean IsAtCompetition) {

        moss = new SparkMax(ElevID, MotorType.kBrushless);
        Joystick = Controller; 
        IsAtComp = IsAtCompetition;

        mossConfig.voltageCompensation(10);
        mossConfig.smartCurrentLimit(60);
        mossConfig.idleMode(IdleMode.kBrake);
        mossConfig.apply(mossCCLoop);

        moss.configure(mossConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        driveMoss(Joystick);
    }

    public boolean DeadbandCheck(double Value) {
        return Math.abs(Value) > deadPan;
    }

    public boolean mossHeightBounds(double controllerY) {
        return !(moss.getEncoder().getPosition() - 8 * controllerY < Constants.kElevatorLowerBound ||
                moss.getEncoder().getPosition() - 8 * controllerY >= Constants.kElevatorUpperBound);
    }

    public Command driveMoss (CommandXboxController Controller) {

        if (DeadbandCheck(Controller.getLeftY()) && mossHeightBounds(Controller.getLeftY())) {
            moss.set(-Controller.getLeftY());
        } else {
            moss.stopMotor();
        }

        return Commands.none();
    }
}