package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Intake tuning komutu.
 * Extension mesafesi (cm) ve Roller RPM dashboard'dan girilerek canlı tuning
 * yapılır.
 * 
 * <h2>Dashboard Yolları:</h2>
 * <ul>
 * <li><code>/Tuning/Intake/Test/Extension Cm</code> - Extension hedef mesafesi
 * (cm)</li>
 * <li><code>/Tuning/Intake/Test/Roller RPM</code> - Roller hedef hızı
 * (RPM)</li>
 * </ul>
 */
public class IntakeTestCommand extends Command {

    private final IntakeSubsystem intake;

    private final TunableNumber testExtensionCm;
    private final TunableNumber testRollerRPM;

    public IntakeTestCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);

        testExtensionCm = new TunableNumber("Tuning/Intake/Test", "Extension Cm", 0.0);
        testRollerRPM = new TunableNumber("Tuning/Intake/Test", "Roller RPM", 0.0);
    }

    @Override
    public void initialize() {
        System.out.println("========================================");
        System.out.println("[IntakeTest] Test modu başlatıldı!");
        System.out.println("[IntakeTest] Extension Cm: /Tuning/Intake/Test/Extension Cm");
        System.out.println("[IntakeTest] Roller RPM: /Tuning/Intake/Test/Roller RPM");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        // =========== EXTENSION (cm) ===========
        double extensionTarget = testExtensionCm.get();
        intake.setExtensionPosition(extensionTarget);

        // =========== ROLLER (RPM) ===========
        double rollerTarget = testRollerRPM.get();
        if (Math.abs(rollerTarget) > 10) {
            intake.runRollerRPM(rollerTarget);
        } else {
            intake.stopRoller();
        }

        // =========== LOG ===========
        Logger.recordOutput("Tuning/Intake/Test/TargetExtensionCm", extensionTarget);
        Logger.recordOutput("Tuning/Intake/Test/ActualExtensionCm", intake.getExtensionPositionCm());
        Logger.recordOutput("Tuning/Intake/Test/TargetRollerRPM", rollerTarget);
        Logger.recordOutput("Tuning/Intake/Test/ActualRollerRPM", intake.getRollerRPM());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopAll();
        System.out.println("[IntakeTest] Test modu sonlandırıldı");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
