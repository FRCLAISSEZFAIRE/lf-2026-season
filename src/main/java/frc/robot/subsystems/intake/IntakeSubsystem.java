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

    // Nesne sayacı
    private int itemCount = 0;

    // Edge detection için önceki sensör durumu
    private boolean lastIntakeSensorState = false;

    // Aktif oyun nesnesi
    private GamePiece activeGamePiece = GamePieceConstants.DEFAULT_GAME_PIECE;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // --- EDGE DETECTION: Intake Sensör ---
        // Sensör LOW'dan HIGH'a geçtiyse (nesne geçti)
        if (inputs.intakeSensorTriggered && !lastIntakeSensorState) {
            if (itemCount < MechanismConstants.kMaxItemCount) {
                itemCount++;
            }
        }
        lastIntakeSensorState = inputs.intakeSensorTriggered;

        // --- LOGLAMA ---
        Logger.recordOutput("Intake/ItemCount", itemCount);
        Logger.recordOutput("Intake/IsFull", isFull());
        Logger.recordOutput("Intake/SensorTriggered", inputs.intakeSensorTriggered);
        
        // Dashboard Indicator (User Request)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Fuel Present", seesGamePiece());
    }

    /**
     * Intake motorunu çalıştırır.
     * Eğer kapasite doluysa çalışmaz.
     */
    public void runRoller(double volts) {
        if (isFull() && volts > 0) {
            // Kapasite dolu, intake çalışmasın
            io.setVoltage(0);
        } else {
            io.setVoltage(volts);
        }
    }

    /**
     * Shooter'dan nesne çıktığında çağrılır.
     * Item sayacını azaltır.
     */
    public void decrementItemCount() {
        if (itemCount > 0) {
            itemCount--;
        }
    }

    /**
     * Nesne sayacını manuel olarak ayarlar (test için).
     */
    public void setItemCount(int count) {
        this.itemCount = Math.max(0, Math.min(count, MechanismConstants.kMaxItemCount));
    }

    /**
     * Kapasite dolu mu?
     */
    public boolean isFull() {
        return itemCount >= MechanismConstants.kMaxItemCount;
    }

    /**
     * Mevcut nesne sayısını döndürür.
     */
    public int getItemCount() {
        return itemCount;
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