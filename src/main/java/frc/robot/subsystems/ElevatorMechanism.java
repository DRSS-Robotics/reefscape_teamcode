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

    ClosedLoopConfig elevCCLoop = new ClosedLoopConfig()
            .p(Constants.kElevatorKp).i(Constants.kElevatorKi).d(Constants.kElevatorKd);

    SparkMaxConfig elevConfig = new SparkMaxConfig();

    public final double deadband = 0.04;
    public final SparkMax elevatorMotor;
    CommandXboxController joystick;
    
    // indicates whether closed-loop control should use our practice field setpoints
    // or the actual competition values.
    public boolean isAtComp;

    public ElevatorMechanism(int ElevID, CommandXboxController Controller, boolean IsAtCompetition) {

        elevatorMotor = new SparkMax(ElevID, MotorType.kBrushless);
        joystick = Controller;
        isAtComp = IsAtCompetition;

        elevConfig.voltageCompensation(10);
        elevConfig.smartCurrentLimit(60);
        elevConfig.idleMode(IdleMode.kBrake);
        elevConfig.apply(elevCCLoop);

        elevatorMotor.configure(elevConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        driveElevator(joystick);
    }

    public boolean deadbandCheck(double Value) {
        return Math.abs(Value) > deadband;
    }

    public boolean motorHeightBounds(double controllerY) {
        return !(elevatorMotor.getEncoder().getPosition() - 8 * controllerY < Constants.kElevatorLowerBound ||
                elevatorMotor.getEncoder().getPosition() - 8 * controllerY >= Constants.kElevatorUpperBound);
    }

    public Command driveElevator(CommandXboxController controller) {

        if (deadbandCheck(controller.getLeftY()) && motorHeightBounds(controller.getLeftY())) {
            elevatorMotor.set(-controller.getLeftY());
        } else {
            elevatorMotor.stopMotor();
        }

        return Commands.none();
    }

}
