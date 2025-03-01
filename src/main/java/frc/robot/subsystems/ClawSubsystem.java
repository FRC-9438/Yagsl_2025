<<<<<<< HEAD
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
=======
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class ClawSubsystem extends SubsystemBase {

    // Motor controller
    private final SparkMax clawMotor;
    
    // PID controller
    private final SparkClosedLoopController clawPID;
  
    // PID coefficients
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
  
    // Setpoints
    private static final double UNLOAD_POSITION  = 0.7;
    private static final double LOAD_POSITION = -1.5; // Example value
  
    public ClawSubsystem() {
        // Initialize motor on CAN ID 60, brushless
        clawMotor = new SparkMax(60, MotorType.kBrushless);
  
        // Create a new configuration object
        SparkMaxConfig config = new SparkMaxConfig();
  
        // Set desired configurations
        config.inverted(false); // Set according to your hardware setup
        config.closedLoop.pid(kP, kI, kD);
        config.closedLoop.outputRange(-1.0, 1.0);
  
        // Apply the configuration with a safe reset and persist the parameters
        clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
        // Retrieve the PID controller
        clawPID = clawMotor.getClosedLoopController();
    }
  
    // Set the claw position
    public void setPosition(double position) {
        clawPID.setReference(position, SparkMax.ControlType.kPosition);
    }
  
    // Manual control
    public void setPower(double percentOutput) {
        clawMotor.set(percentOutput);
    }
}
>>>>>>> b69cc1194aabe1b3d95f6ccfc01fb3b8c99624d2
