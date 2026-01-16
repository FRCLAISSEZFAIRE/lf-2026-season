package frc.robot.constants;

/**
 * Oyun nesnelerinin sabit değerleri.
 */
public final class GamePieceConstants {

    /**
     * Oyun nesnesi tanımı.
     * 
     * @param id          Limelight class ID'si
     * @param intakeSpeed Intake motor voltajı
     * @param name        Görüntüleme adı
     */
    public record GamePiece(int id, double intakeSpeed, String name) {
    }

    /**
     * Mevcut oyun nesneleri listesi.
     * Limelight Neural Network pipeline'da bu ID'ler class olarak tanımlı olmalı.
     */
    public static final GamePiece[] GAME_PIECES = {
            new GamePiece(0, 10.0, "Fuel"), // 2026 Fuel
    };

    /**
     * Varsayılan oyun nesnesi (ilk eleman)
     */
    public static final GamePiece DEFAULT_GAME_PIECE = GAME_PIECES[0];

    /**
     * ID'ye göre oyun nesnesi bul.
     * 
     * @param id Aranacak ID
     * @return Bulunan GamePiece veya varsayılan
     */
    public static GamePiece getById(int id) {
        for (GamePiece piece : GAME_PIECES) {
            if (piece.id() == id) {
                return piece;
            }
        }
        return DEFAULT_GAME_PIECE;
    }

    /**
     * İsme göre oyun nesnesi bul.
     * 
     * @param name Aranacak isim
     * @return Bulunan GamePiece veya varsayılan
     */
    public static GamePiece getByName(String name) {
        for (GamePiece piece : GAME_PIECES) {
            if (piece.name().equalsIgnoreCase(name)) {
                return piece;
            }
        }
        return DEFAULT_GAME_PIECE;
    }

    // TOF Sensör Eşik Değeri (mm)
    public static final double kTofDetectionThresholdMm = 100.0;

    // Auto-Intake PID Sabitleri
    public static final double kAlignP = 0.03;
    public static final double kAlignI = 0.0;
    public static final double kAlignD = 0.001;
    public static final double kAlignTolerance = 2.0; // derece

    // Auto-Intake Hız Sabitleri
    public static final double kApproachSpeed = 1.5; // m/s
    public static final double kMinTargetArea = 0.5; // % - hedef çok küçükse durma
}
