package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * Arm'ı belirtilen açıya götürür.
 */
public class SetArmAngleCommand extends Command {

    private final ArmSubsystem armSubsystem;
    private final double targetAngle;

    /**
     * @param armSubsystem Arm alt sistemi
     * @param angleDegrees Hedef açı (derece)
     */
    public SetArmAngleCommand(ArmSubsystem armSubsystem, double angleDegrees) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = angleDegrees;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            armSubsystem.stop();
        }
    }
}
