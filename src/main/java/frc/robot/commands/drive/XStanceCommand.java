package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Tekerlekleri X formasyonuna getirip robotu kilitler.
 * Savunma veya fren için kullanılır.
 */
public class XStanceCommand extends Command {

    private final DriveSubsystem driveSubsystem;

    public XStanceCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Tuşa basılı tutulduğu sürece çalışır
    }
}
