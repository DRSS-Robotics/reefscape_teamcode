package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

public class hangSubsystem { 
    private int motorID;
    private double speed;    

    //constructor to initialize the motor's ID and speed
    public DCMotor(int motorID, double speed) {
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

    //turn the motor on
    public static void motorOn(DCMotor motor) {
        motor.setSpeed(1.0); //set motor speed to full speed
    }

    //main method to test the hangSubsystem
    public static void main(String[] args) {
        hangSubsystem subsystem = new hangSubsystem();
        DCMotor motor = subsystem.new DCMotor(1, 0.0); //create a new DCMotor with ID 1 and initial speed 0.0

        //turn the motor on
        motorOn(motor);
    }
}




