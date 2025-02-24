package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax clawMotor1;
    private final SparkMax clawMotor2;

    public ClawSubsystem() {
        // Initialize motors with their respective CAN IDs
        clawMotor1 = new SparkMax(10, MotorType.kBrushless);
        clawMotor2 = new SparkMax(11, MotorType.kBrushless);

        // Create and configure the SparkMaxConfig for clawMotor1
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.inverted(false); // Set to 'true' if motor direction needs to be inverted
        clawMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Create and configure the SparkMaxConfig for clawMotor2
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(true); // Set to 'false' if motor direction is correct
        clawMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to control the climbing mechanism
    public void Intake(double speed) {
        clawMotor1.set(speed);
        clawMotor2.set(speed);
    }

    // Method to stop the climbing mechanism
    public void stopIntake() {
        clawMotor1.set(0);
        clawMotor2.set(0);
    }
}
