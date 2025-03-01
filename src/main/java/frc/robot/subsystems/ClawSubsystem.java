package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax clawMotor;

    public ClawSubsystem() {
        // Initialize motors with their respective CAN IDs
        clawMotor = new SparkMax(54, MotorType.kBrushless); //CAN ID 54

        // Create and configure the SparkMaxConfig for clawMotor1
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.inverted(false); // Set to 'true' if motor direction needs to be inverted
        clawMotor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to control the climbing mechanism
    public void Intake(double speed) {
        clawMotor.set(speed);
    }

    // Method to stop the climbing mechanism
    public void stopIntake() {
        clawMotor.set(0);
    }
}
