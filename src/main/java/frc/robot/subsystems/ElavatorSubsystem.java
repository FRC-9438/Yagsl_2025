package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElavatorSubsystem extends SubsystemBase {
    private final SparkMax elavatorMotor1;
    private final SparkMax elavatorMotor2;


    

    public ElavatorSubsystem() {
        elavatorMotor1 = new SparkMax(6, MotorType.kBrushless); // CAN ID 6
        elavatorMotor2 = new SparkMax(7, MotorType.kBrushless); // CAN ID 7

        // Additional motor configuration if needed
    }

    public void moveArm(double speed) {
        elavatorMotor1.set(speed);
        elavatorMotor2.set(speed);

    }

    public void stopArm() {
        elavatorMotor1.set(0);
        elavatorMotor2.set(0);
    }

    @Override
    public void periodic() {
        // Code to run periodically
    }
}
