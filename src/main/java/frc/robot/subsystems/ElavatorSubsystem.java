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

public class ElavatorSubsystem extends SubsystemBase {
    private final SparkMax elavatorMotor1;
    private final SparkMax elavatorMotor2;
    private final RelativeEncoder elavatorEncoder;
    private final SparkClosedLoopController elavatorPID;
    private final SparkMaxConfig motorConfig;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMinOutput = -1.0;
    private static final double kMaxOutput = 1.0;
    private static final double kConversionFactor = 0.5;

    public static final double L1 = 0.0;
    public static final double L2 = 20.0;
    public static final double L3 = 40.0;
    public static final double L4 = 60.0;

    public ElavatorSubsystem() {
        // Initialize motors
        elavatorMotor1 = new SparkMax(50, MotorType.kBrushless);
        elavatorMotor2 = new SparkMax(51, MotorType.kBrushless);

        // Get encoder from motor 1
        elavatorEncoder = elavatorMotor1.getEncoder();

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
        elavatorMotor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get PID controller
        elavatorPID = elavatorMotor1.getClosedLoopController();

        // Reset encoder to 0 at startup
        elavatorEncoder.setPosition(0.0);
    }

    public void moveArm(double speed) {
        elavatorMotor1.set(speed);
        elavatorMotor2.set(speed);
    }

    public void stopArm() {
        elavatorMotor1.set(0.0);
        elavatorMotor2.set(0.0);
    }

    public void setPosition(double rotations) {
        elavatorPID.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPosition() {
        return elavatorEncoder.getPosition();
    }

    public boolean atSetpoint(double target, double tolerance) {
        return Math.abs(getPosition() - target) <= tolerance;
    }
}
