package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Robotu yukarı çeker (Retract).
 * Seat sensor tetiklenene kadar veya hedef pozisyona ulaşana kadar çalışır.
 */
public class ClimberRetractCommand extends Command {
    private final ClimberSubsystem climberSubsystem;

    public ClimberRetractCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        // Ya hedef pozisyona ulaştık (encoder)
        // YA DA sensör tetiklendi (güvenli oturuş)
        return climberSubsystem.isAtTarget() || climberSubsystem.isSeated();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climberSubsystem.stop();
        }
        // Normal bitişte motorlar hold modunda kalır
    }
}
