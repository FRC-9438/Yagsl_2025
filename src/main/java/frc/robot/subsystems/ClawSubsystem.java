package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax clawMotor;
    private final RelativeEncoder clawEncoder;
    private final SparkClosedLoopController clawPID;
    private final SparkMaxConfig motorConfig;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMinOutput = -1.0;
    private static final double kMaxOutput = 1.0;
    private static final double kConversionFactor = 0.5;

    public static final double Intake = 2.2;
    public static final double Stop = 0;
  

    public ClawSubsystem() {
        // Initialize motors
        clawMotor = new SparkMax(60, MotorType.kBrushless);

        // Get encoder from motor 1
        clawEncoder = clawMotor.getEncoder();

        // Create motor configuration object
        motorConfig = new SparkMaxConfig();

        // Configure encoder conversion factors inside motorConfig
        motorConfig.encoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor);

        // Configure the closed-loop PID controller
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // Use internal encoder
            .p(kP, ClosedLoopSlot.kSlot0)  // Set PID for position (slot 0)
            .i(kI, ClosedLoopSlot.kSlot0)
            .d(kD, ClosedLoopSlot.kSlot0)
            .outputRange(kMinOutput, kMaxOutput, ClosedLoopSlot.kSlot0);  // Set output range

        // Apply configuration to Spark MAX
        clawMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get PID controller
        clawPID = clawMotor.getClosedLoopController();

        // Reset encoder to 0 at startup
        clawEncoder.setPosition(0.0);
    }

    public void Intake(double speed) {
        clawMotor.set(speed);
    }

    public void stopIntake() {
        clawMotor.set(0.0);
    }

    public void setPosition(double rotations) {
        clawPID.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPosition() {
        return clawEncoder.getPosition();
    }

    public boolean atSetpoint(double target, double tolerance) {
        return Math.abs(getPosition() - target) <= tolerance;
    }
}
