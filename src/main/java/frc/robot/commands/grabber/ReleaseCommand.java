package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

/**
 * Grabber'ı açar (oyun parçasını bırakır).
 */
public class ReleaseCommand extends Command {

    private final GrabberSubsystem grabberSubsystem;

    public ReleaseCommand(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.open();
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
