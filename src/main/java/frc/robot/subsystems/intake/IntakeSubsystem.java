package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.GamePieceConstants;
import frc.robot.constants.GamePieceConstants.GamePiece;

/**
 * Intake alt sistemi.
 * MZ80 sensör ile nesne sayımı yapar.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Pivot Constants
    private static final double kPivotDeployedRad = 1.5; // Örnek Açı (~85 derece)
    private static final double kPivotRetractedRad = 0.0; // Kapalı Konum
    private static final double kPivotToleranceRad = 0.1;

    private GamePiece activeGamePiece = GamePieceConstants.DEFAULT_GAME_PIECE; // Varsayılan

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // --- LOGLAMA ---
        Logger.recordOutput("Intake/PivotPosition", inputs.pivotPositionRad);
        // Dashboard Indicator
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Fuel Present", seesGamePiece());
    }

    /**
     * Intake motorunu çalıştırır.
     */
    public void runRoller(double volts) {
        io.setVoltage(volts);
    }

    // ==================== PIVOT CONTROL ====================

    public void setPivotPosition(double rad) {
        io.setPivotPosition(rad);
    }

    public void deploy() {
        setPivotPosition(kPivotDeployedRad);
    }

    public void retract() {
        setPivotPosition(kPivotRetractedRad);
    }

    public boolean isDeployed() {
        return Math.abs(inputs.pivotPositionRad - kPivotDeployedRad) < kPivotToleranceRad;
    }

    /**
     * Kapasite dolu mu? (Artık kullanılmıyor, Feeder kontrol ediyor)
     */
    public boolean isFull() {
        return false;
    }

    /**
     * Mevcut nesne sayısını döndürür.
     */
    public int getItemCount() {
        return 0; // Sensör iptal edildi
    }

    public void decrementItemCount() {
        // Nesne atıldı (mantıksal takip gerekirse buraya eklenecek)
    }

    /** Robotun oyun parçasına hizalanması için gereken açı hatasını döndürür */
    public double getAlignmentError() {
        if (seesActiveGamePiece()) {
            return inputs.targetTx;
        }
        return 0.0;
    }

    public boolean seesGamePiece() {
        return inputs.hasGamePiece;
    }

    // ==================== GAME PIECE TRACKING ====================

    /**
     * Aktif oyun nesnesini ayarlar.
     * Limelight sadece bu nesneyi takip edecek.
     */
    public void setActiveGamePiece(GamePiece piece) {
        this.activeGamePiece = piece;
        Logger.recordOutput("Intake/ActiveGamePiece", piece.name());
    }

    /**
     * Aktif oyun nesnesini döndürür.
     */
    public GamePiece getActiveGamePiece() {
        return activeGamePiece;
    }

    /**
     * Kameranın aktif oyun nesnesini görüp görmediğini kontrol eder.
     * Sadece activeGamePiece ID'sine uyan hedefleri kabul eder.
     */
    public boolean seesActiveGamePiece() {
        return inputs.hasGamePiece && inputs.targetClassId == activeGamePiece.id();
    }

    /**
     * TOF sensörüne göre intake içinde nesne var mı?
     */
    public boolean hasGamePieceInIntake() {
        return inputs.tofDistanceMm < GamePieceConstants.kTofDetectionThresholdMm;
    }

    /**
     * Hedefin yatay açı hatasını döndürür (derece).
     */
    public double getTargetTx() {
        return inputs.targetTx;
    }

    /**
     * Hedefin dikey açı hatasını döndürür (derece).
     */
    public double getTargetTy() {
        return inputs.targetTy;
    }

    /**
     * Hedefin alanını döndürür (% ekran alanı).
     */
    public double getTargetArea() {
        return inputs.targetArea;
    }

    /**
     * Aktif oyun nesnesinin intake hızını döndürür.
     */
    public double getActiveIntakeSpeed() {
        return activeGamePiece.intakeSpeed();
    }
}