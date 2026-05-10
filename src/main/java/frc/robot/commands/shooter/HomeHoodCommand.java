package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command to HOME the Hood.
 * Drives the hood motor downward for 1 second,
 * then resets the encoder to 0 (mechanical home).
 * 
 * This is a thin wrapper around ShooterSubsystem.getHomeHoodCommand().
 */
public class HomeHoodCommand extends Command {
    private final ShooterSubsystem shooter;
    private Command innerCommand;

    public HomeHoodCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        innerCommand = shooter.getHomeHoodCommand();
        innerCommand.initialize();
        System.out.println("[HomeHood] Starting Hood Homing...");
    }

    @Override
    public void execute() {
        innerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return innerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        innerCommand.end(interrupted);
    }
}
