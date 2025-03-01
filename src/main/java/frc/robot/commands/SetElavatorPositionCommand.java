package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElavatorSubsystem;

public class SetElavatorPositionCommand extends Command {
    private final ElavatorSubsystem elavatorSubsystem;
    private final double targetPosition;
    private final double tolerance;

    public SetElavatorPositionCommand(ElavatorSubsystem subsystem, double targetPosition, double tolerance) {
        this.elavatorSubsystem = subsystem;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elavatorSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return elavatorSubsystem.atSetpoint(targetPosition, tolerance);
    }
}
