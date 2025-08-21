package frc.robot.subsystems.CoralMechanism;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralMechanismIOSparkMax {
    private final SparkMax coralMotor;

    public CoralMechanismIOSparkMax(int CoralID){
        coralMotor = new SparkMax(CoralID, MotorType.kBrushless);
    }
}
