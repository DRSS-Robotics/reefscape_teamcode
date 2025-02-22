package frc.robot;

public class MotorRequest {
    private final double targetPosition;

    public MotorRequest(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}

