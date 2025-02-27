package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class CommandHangSubsystem extends SubsystemBase { 
    private int motorID;
    private double speed;   

    private SparkMax hangMotor = new SparkMax(1, MotorType.kBrushed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    private MyDCMotor motor; 




    public void motorOn(DCMotor motor) {
        this.motor.setSpeed(1.0); // Set motor to full speed
        System.out.println("Motor is now on at speed: " + this.motor.getSpeed());
    }

    //constructor to initialize the motor's ID and speed
    public CommandHangSubsystem(int motorID, double speed) {
        this.motorID = motorID;
        this.speed = speed;
        XboxController gamepad2 = new XboxController(motorID);
    }
    public void XboxController(){
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}




