package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElavatorSubsystem;

public class ElavatorCommand extends Command {
    private final ElavatorSubsystem elavatorSubsystem;
    private final double speed;

    public ElavatorCommand(ElavatorSubsystem subsystem, double speed) {
        this.elavatorSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem); // Prevents other commands from interrupting
    }

    @Override
    public void execute() {
        elavatorSubsystem.moveArm(speed); // Moves the elevator at the specified speed
    }

    @Override
    public void end(boolean interrupted) {
        elavatorSubsystem.stopArm(); // Stops the motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}
