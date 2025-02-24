package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor1;
    private final SparkMax climbMotor2;

    public ClimbSubsystem() {
        // Initialize motors with their respective CAN IDs
        climbMotor1 = new SparkMax(6, MotorType.kBrushless);
        climbMotor2 = new SparkMax(7, MotorType.kBrushless);

        // Create and configure the SparkMaxConfig for climbMotor1
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.inverted(false); // Set to 'true' if motor direction needs to be inverted
        climbMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Create and configure the SparkMaxConfig for climbMotor2
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(true); // Set to 'false' if motor direction is correct
        climbMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to control the climbing mechanism
    public void climb(double speed) {
        climbMotor1.set(speed);
        climbMotor2.set(speed);
    }

    // Method to stop the climbing mechanism
    public void stopClimb() {
        climbMotor1.set(0);
        climbMotor2.set(0);
    }
}
