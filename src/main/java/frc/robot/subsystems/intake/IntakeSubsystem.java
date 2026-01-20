package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.GamePieceConstants;
import frc.robot.constants.GamePieceConstants.GamePiece;
import frc.robot.util.TunableNumber;

/**
 * Intake alt sistemi.
 * TunableNumber ile runtime'da ayarlanabilen parametreler.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Pivot Constants
    private static final double kPivotToleranceRad = 0.1;

    private GamePiece activeGamePiece = GamePieceConstants.DEFAULT_GAME_PIECE;

    // ===========================================================================
    // TUNABLE PARAMETERS - Kalıcı olarak kaydedilir
    // ===========================================================================
    private final TunableNumber tunablePivotDeployed;
    private final TunableNumber tunablePivotRetracted;
    private final TunableNumber tunablePivotP;
    private final TunableNumber tunablePivotI;
    private final TunableNumber tunablePivotD;
    private final TunableNumber tunableRollerSpeed;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;

        // TunableNumber oluştur - Preferences'ta kalıcı
        tunablePivotDeployed = new TunableNumber("Intake", "Pivot Deployed Rad", 1.5);
        tunablePivotRetracted = new TunableNumber("Intake", "Pivot Retracted Rad", 0.0);
        tunablePivotP = new TunableNumber("Intake", "Pivot P", 1.0);
        tunablePivotI = new TunableNumber("Intake", "Pivot I", 0.0);
        tunablePivotD = new TunableNumber("Intake", "Pivot D", 0.0);
        tunableRollerSpeed = new TunableNumber("Intake", "Roller Speed V", 8.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Tunable PID değişiklik kontrolü
        if (tunablePivotP.hasChanged() || tunablePivotI.hasChanged() || tunablePivotD.hasChanged()) {
            if (io instanceof IntakeIOReal) {
                ((IntakeIOReal) io).configurePivotPID(
                        tunablePivotP.get(),
                        tunablePivotI.get(),
                        tunablePivotD.get());
            }
        }

        // Logging
        Logger.recordOutput("Intake/PivotPosition", inputs.pivotPositionRad);
        Logger.recordOutput("Intake/TunablePivotP", tunablePivotP.get());
        Logger.recordOutput("Intake/TunableRollerSpeed", tunableRollerSpeed.get());

        // Dashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Fuel Present", seesGamePiece());
    }

    // ===========================================================================
    // ROLLER CONTROL
    // ===========================================================================

    public void runRoller(double volts) {
        io.setVoltage(volts);
    }

    /** Tunable hızla roller çalıştır */
    public void runRollerDefault() {
        io.setVoltage(tunableRollerSpeed.get());
    }

    /** Roller'ı ters çalıştır */
    public void reverseRoller() {
        io.setVoltage(-tunableRollerSpeed.get());
    }

    public void stopRoller() {
        io.setVoltage(0);
    }

    // ===========================================================================
    // PIVOT CONTROL
    // ===========================================================================

    public void setPivotPosition(double rad) {
        io.setPivotPosition(rad);
    }

    public void deploy() {
        setPivotPosition(tunablePivotDeployed.get());
    }

    public void retract() {
        setPivotPosition(tunablePivotRetracted.get());
    }

    public boolean isDeployed() {
        return Math.abs(inputs.pivotPositionRad - tunablePivotDeployed.get()) < kPivotToleranceRad;
    }

    public boolean isRetracted() {
        return Math.abs(inputs.pivotPositionRad - tunablePivotRetracted.get()) < kPivotToleranceRad;
    }

    // ===========================================================================
    // CAPACITY & ITEM COUNT
    // ===========================================================================

    public boolean isFull() {
        return false;
    }

    public int getItemCount() {
        return 0;
    }

    public void decrementItemCount() {
    }

    // ===========================================================================
    // ALIGNMENT & VISION
    // ===========================================================================

    public double getAlignmentError() {
        return seesActiveGamePiece() ? inputs.targetTx : 0.0;
    }

    public boolean seesGamePiece() {
        return inputs.hasGamePiece;
    }

    public void setActiveGamePiece(GamePiece piece) {
        this.activeGamePiece = piece;
        Logger.recordOutput("Intake/ActiveGamePiece", piece.name());
    }

    public GamePiece getActiveGamePiece() {
        return activeGamePiece;
    }

    public boolean seesActiveGamePiece() {
        return inputs.hasGamePiece && inputs.targetClassId == activeGamePiece.id();
    }

    public boolean hasGamePieceInIntake() {
        return inputs.tofDistanceMm < GamePieceConstants.kTofDetectionThresholdMm;
    }

    public double getTargetTx() {
        return inputs.targetTx;
    }

    public double getTargetTy() {
        return inputs.targetTy;
    }

    public double getTargetArea() {
        return inputs.targetArea;
    }

    public double getActiveIntakeSpeed() {
        return activeGamePiece.intakeSpeed();
    }

    // ===========================================================================
    // TUNABLE GETTERS
    // ===========================================================================

    public double getTunableRollerSpeed() {
        return tunableRollerSpeed.get();
    }

    public double getTunablePivotDeployed() {
        return tunablePivotDeployed.get();
    }

    public double getTunablePivotRetracted() {
        return tunablePivotRetracted.get();
    }
}