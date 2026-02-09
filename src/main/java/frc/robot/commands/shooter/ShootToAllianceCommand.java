package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

import java.util.function.Supplier;

/**
 * Shoot to Alliance Pass Target Command.
 * 
 * - Updates Aiming for Alliance Pass (Feeding Station) based on Robot Pose.
 * - Feeder ONLY runs when shooter is READY.
 */
public class ShootToAllianceCommand extends Command {

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Supplier<Pose2d> poseSupplier;
    private boolean hasShot = false;

    public ShootToAllianceCommand(
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
        System.out.println("[ShootToAllianceCommand] Started - Targeting Alliance Pass");
    }

    @Override
    public void execute() {
        Pose2d currentPose = poseSupplier.get();
        if (currentPose == null) {
            feeder.stop();
            return;
        }

        // Switch aiming logic to Alliance Pass
        shooter.updateAimingForPass(currentPose);

        if (shooter.isReadyToShoot()) {
            if (!hasShot) {
                hasShot = true;
            }
            if (!feeder.isLoading()) {
                feeder.feed();
            }
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        feeder.stop();
        System.out.println("[ShootToAllianceCommand] Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
