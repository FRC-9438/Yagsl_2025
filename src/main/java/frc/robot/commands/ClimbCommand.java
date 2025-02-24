package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final double speed;

    public ClimbCommand(ClimbSubsystem subsystem, double speed) {
        this.climbSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem); // Prevents other commands from interrupting
    }

    @Override
    public void execute() {
        climbSubsystem.climb(speed); // Moves the climbing mechanism at the specified speed
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopClimb(); // Stops the motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}
