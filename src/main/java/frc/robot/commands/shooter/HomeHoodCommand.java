package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command to HOME the Hood.
 * Drives the hood motor to the bottom mechanical stop,
 * then resets the encoder to 30.0 (mechanical home).
 */
public class HomeHoodCommand extends Command {
    private final ShooterSubsystem shooter;
    private double startTime;

    public HomeHoodCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(30);
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        System.out.println("[HomeHood] Starting Homing...");
    }

    @Override
    public void execute() {
        shooter.setHoodVoltage(-8.0); // Drive down gently\
    }

    @Override
    public boolean isFinished() {
        return (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime) >= 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodVoltage(0);
        if (!interrupted) {
            shooter.resetHoodEncoder();
            System.out.println("[HomeHood] Hood Homed & Encoder Reset to physical home (30°).");
        }
    }
}
