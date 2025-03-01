package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends Command {
    private final ClawSubsystem clawSubsystem;
    private final double targetPosition;
    private final double tolerance;

    public ClawCommand(ClawSubsystem subsystem, double targetPosition, double tolerance) {
        this.clawSubsystem = subsystem;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return clawSubsystem.atSetpoint(targetPosition, tolerance);
    }
}
