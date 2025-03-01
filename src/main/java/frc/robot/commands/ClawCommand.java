package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends Command {
    private final ClawSubsystem clawSubsystem;
    private final double speed;

    public ClawCommand(ClawSubsystem subsystem, double speed) {
        this.clawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem); // Prevents other commands from interrupting
    }

    @Override
    public void execute() {
        clawSubsystem.Intake(speed); // Moves the climbing mechanism at the specified speed
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopIntake(); // Stops the motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}
