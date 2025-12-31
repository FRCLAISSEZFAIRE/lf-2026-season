package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Climber'ı aşağı indirir.
 * Basılı tutulduğu sürece çalışır.
 */
public class ClimbDownCommand extends Command {

    private final ClimberSubsystem climberSubsystem;

    public ClimbDownCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.climbDown();
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
