package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandHangSubsystem extends SubsystemBase { 
    private int motorID;
    private double speed;   


    private MyDCMotor motor; 

    public void motorOn(DCMotor motor) {
        this.motor.setSpeed(1.0); // Set motor to full speed
        System.out.println("Motor is now on at speed: " + this.motor.getSpeed());
    }

    //constructor to initialize the motor's ID and speed
    public void motor(int motorID, double speed) {
        this.motorID = motorID;
        this.speed = speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    //get the current speed of the motor
    public double getSpeed() {
        return speed;
    }

    //get the motor's ID
    public int getMotorID() {
        return motorID;
    }
    
    public void motorOff(DCMotor motor) {
        this.motor(1,0); //sets motor to full speed
     }

    //main method to test the hangSubsystem
    // public static void main(String[] args) {
    //     hangSubsystem subsystem = new hangSubsystem();
    //     motor HangMotor = new motor(1, 0.0); //create a new DCMotor with ID 1 and initial speed 0.0 [THIS IS WHERE THE PORT WOULD BE MODIFIED]

    //     //turn the motor on
    // }
}




