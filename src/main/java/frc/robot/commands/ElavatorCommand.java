package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElavatorSubsystem;

public class ElavatorCommand extends Command {
    private final ElavatorSubsystem elavatorSubsystem;
    private final double speed;

    public ElavatorCommand(ElavatorSubsystem subsystem, double speed) {
        this.elavatorSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        elavatorSubsystem.moveArm(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elavatorSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
