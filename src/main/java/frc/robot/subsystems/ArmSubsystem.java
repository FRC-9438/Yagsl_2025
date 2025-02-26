package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;

    public ArmSubsystem() {
        armMotor = new SparkMax(60, MotorType.kBrushless); // CAN ID 60
        // Additional motor configuration if needed
    }

    public void moveArm(double speed) {
        armMotor.set(speed);
    }

    public void stopArm() {
        armMotor.set(0);
    }

    @Override
    public void periodic() {
        // Code to run periodically
    }
}
