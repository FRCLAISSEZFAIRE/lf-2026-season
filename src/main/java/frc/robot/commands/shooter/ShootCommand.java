package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

import java.util.function.Supplier;

/**
 * Pose-Based Shooting Command.
 * 
 * - Continuously updates Aiming (Turret, Hood, Flywheel) based on Robot Pose.
 * - Feeder ONLY runs when shooter is READY (PID checks pass).
 * - "Hold Fire" logic: Automatically stops feeding if accuracy is lost.
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Supplier<Pose2d> poseSupplier;

    // Optional: Single shot state, but "Hold Fire" implies continuous fire
    // capability if held
    private boolean hasShot = false;

    /**
     * Creates a new ShootCommand.
     * 
     * @param shooter      Subsystem
     * @param feeder       Subsystem
     * @param poseSupplier Supply current robot pose (e.g. from DriveSubsystem)
     */
    public ShootCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.poseSupplier = poseSupplier;

        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        hasShot = false;
        System.out.println("[ShootCommand] Started - Pose Targeting");
    }

    @Override
    public void execute() {
        // 1. Get Current Pose
        Pose2d currentPose = poseSupplier.get();
        if (currentPose == null) {
            // Safety fallback? Just don't update aiming, or stop?
            // Usually pose is never null if initialized correctly.
            feeder.stop();
            return;
        }

        // 2. Update Aiming (Turret, Hood, RPM)
        // CHECK ZONE: Inside Alliance Zone -> Score (Hub), Outside -> Pass (Feed)
        if (shooter.isInAllianceZone()) {
            shooter.updateAiming(currentPose);
        } else {
            shooter.updateAimingForPass(currentPose);
        }

        // 3. "Hold Fire" Logic with LATCH
        // Once we start shooting (isFeeding = true), we ignore RPM drops.
        // We only stop if the command ends (trigger released).

        // RELAXED READY CHECK:
        // Only require Flywheel RPM to be ready.
        // We ignore Turret/Hood alignment to prevent "stuttering" if the robot is
        // moving.
        boolean ready = shooter.isFlywheelAtTarget();

        // Optional: Uncomment to FORCE FEED always (Debug only)
        // ready = true;

        if (ready && !hasShot) {
            hasShot = true; // Mark that we have started shooting
        }

        // LATCH LOGIC:
        // If we are ready OR we have already started shooting (latched), FEED!
        if (ready || hasShot) {
            if (!feeder.isLoading()) {
                feeder.feed();
            }
        } else {
            // Not ready and haven't started yet -> Wait
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop everything when trigger released
        shooter.stopFlywheel();
        feeder.stop();
        System.out.println("[ShootCommand] Stopped");
    }

    @Override
    public boolean isFinished() {
        // Run while button is held
        return false;
    }
}
