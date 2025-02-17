package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class MyDCMotor {
    private Spark motorController;
    private double speed;

    public MyDCMotor(int pwmPort) {
        motorController = new Spark(pwmPort);
        this.speed = 0.0;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        motorController.set(speed);
    }

    public double getSpeed() {
        return speed;
    }
}
