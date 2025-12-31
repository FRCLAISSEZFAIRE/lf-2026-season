package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPreset;

/**
 * Arm'ı preset pozisyonlardan birine götürür.
 */
public class GoToArmPresetCommand extends Command {

    private final ArmSubsystem armSubsystem;
    private final ArmPreset preset;

    public GoToArmPresetCommand(ArmSubsystem armSubsystem, ArmPreset preset) {
        this.armSubsystem = armSubsystem;
        this.preset = preset;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.goToPreset(preset);
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
