package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

/**
 * Grabber'ı kapatır (oyun parçasını tutar).
 */
public class GrabCommand extends Command {

    private final GrabberSubsystem grabberSubsystem;

    public GrabCommand(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return grabberSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Pozisyonu korur
    }
}
