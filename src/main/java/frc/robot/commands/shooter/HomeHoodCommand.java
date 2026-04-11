package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command to HOME the Hood.
 * Drives the hood motor to the bottom mechanical stop,
 * then resets the encoder to 30.0 (mechanical home).
 */
public class HomeHoodCommand extends Command {
    private final ShooterSubsystem shooter;
    private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    private boolean seatingPhase = false;

    public HomeHoodCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        seatingPhase = false;
        System.out.println("[HomeHood] Phase 1: High-Speed PID to 30°...");
        shooter.setHoodAngle(ShooterConstants.kHoodMinAngle);
        // Önce PID ile 30'a hızlıca yönel
        shooter.setHoodAngle(ShooterConstants.kHoodHomeAngle);
    }

    @Override
    public void execute() {
        if (!seatingPhase) {
            // Eğer hood açısı 33°'nin altına düştüyse veya 0.75 saniye geçtiyse (tıkanma
            // ihtimaline karşı), 2. faza geç
            if (shooter.getHoodAngle() <= 33.0 || timer.hasElapsed(0.75)) {
                seatingPhase = true;
                timer.restart();
                System.out.println("[HomeHood] Phase 2: Seating (-4.0V) for 0.25s...");
            }
        } else {
            // Mekanik stop'a yaslanması için çok kısa süre hafifçe zorla
            shooter.setHoodVoltage(-8.0);
        }
    }

    @Override
    public boolean isFinished() {
        // Yaslanma fazı 0.25 saniye sürdükten sonra komutu bitir
        return seatingPhase && timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodVoltage(0);
        // Seating fazına girdiyse hood mekanik stopa yaklaşmış demektir —
        // interrupt olsa bile encoder'ı resetle (bas-çekte drift birikimini önler)
        if (!interrupted || seatingPhase) {
            shooter.resetHoodEncoder();
            System.out.println("[HomeHood] Hood Homed & Encoder Reset to physical home (30°).");
        }
    }
}
