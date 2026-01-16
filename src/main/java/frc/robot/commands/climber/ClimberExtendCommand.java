package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Tırmanma kancalarını yukarı uzatır (Extend).
 * Hedef pozisyona ulaşana kadar çalışır.
 */
public class ClimberExtendCommand extends Command {
    private final ClimberSubsystem climberSubsystem;

    public ClimberExtendCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.extend();
    }

    @Override
    public void execute() {
        // Hedefe giderken başka bir şey yapmaya gerek yok
        // Subsystem PID ile kontrol ediyor
    }

    @Override
    public boolean isFinished() {
        // Hedef pozisyona ulaştığında komut biter
        return climberSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climberSubsystem.stop();
        }
        // Normal bittiğinde pozisyonda kalır (Hold)
    }
}
