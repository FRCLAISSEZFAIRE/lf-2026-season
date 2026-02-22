package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Intake tuning komutu.
 * Pivot açısı (derece) ve Roller RPM dashboard'dan girilerek canlı tuning
 * yapılır.
 * 
 * <h2>Dashboard Yolları:</h2>
 * <ul>
 * <li><code>/Tuning/Intake/Test/Pivot Angle</code> - Pivot hedef açısı
 * (derece)</li>
 * <li><code>/Tuning/Intake/Test/Roller RPM</code> - Roller hedef hızı
 * (RPM)</li>
 * </ul>
 */
public class IntakeTestCommand extends Command {

    private final IntakeSubsystem intake;

    private final TunableNumber testPivotAngle;
    private final TunableNumber testRollerRPM;

    public IntakeTestCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);

        testPivotAngle = new TunableNumber("Intake/Test", "Pivot Angle", 0.0);
        testRollerRPM = new TunableNumber("Intake/Test", "Roller RPM", 0.0);
    }

    @Override
    public void initialize() {
        System.out.println("========================================");
        System.out.println("[IntakeTest] Test modu başlatıldı!");
        System.out.println("[IntakeTest] Pivot Angle (derece): /Tuning/Intake/Test/Pivot Angle");
        System.out.println("[IntakeTest] Roller RPM: /Tuning/Intake/Test/Roller RPM");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        // =========== PIVOT (derece) ===========
        double pivotTarget = testPivotAngle.get();
        intake.setPivotPosition(pivotTarget);

        // =========== ROLLER (RPM) ===========
        double rollerTarget = testRollerRPM.get();
        if (Math.abs(rollerTarget) > 10) {
            intake.runRollerRPM(rollerTarget);
        } else {
            intake.runRoller(0);
        }

        // =========== LOG ===========
        Logger.recordOutput("Tuning/Intake/Test/TargetPivotDeg", pivotTarget);
        Logger.recordOutput("Tuning/Intake/Test/ActualPivotDeg", intake.getPivotPosition());
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
