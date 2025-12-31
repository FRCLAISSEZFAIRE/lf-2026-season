package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Climber'ı yukarı çeker.
 * Basılı tutulduğu sürece çalışır.
 */
public class ClimbUpCommand extends Command {

    private final ClimberSubsystem climberSubsystem;

    public ClimbUpCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbUp();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
