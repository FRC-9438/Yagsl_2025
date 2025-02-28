package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor1;

    public ClimbSubsystem() {
        // Initialize motors with their respective CAN IDs
        climbMotor1 = new SparkMax(62, MotorType.kBrushless); //CAN ID 62

        // Create and configure the SparkMaxConfig for climbMotor1
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.inverted(false); // Set to 'true' if motor direction needs to be inverted
        climbMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to control the climbing mechanism
    public void climb(double speed) {
        climbMotor1.set(speed);
    }

    // Method to stop the climbing mechanism
    public void stopClimb() {
        climbMotor1.set(0);
    }
}
