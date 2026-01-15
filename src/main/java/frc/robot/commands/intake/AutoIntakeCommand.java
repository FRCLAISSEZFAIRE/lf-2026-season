package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Limelight 3A kullanarak otomatik game piece toplama komutu.
 * - Robotu hedefe yönlendirir (Vision Yaw).
 * - İleri sürer ve Intake/Feeder çalıştırır.
 * - Hedef kaybolursa bir süre (timeout) aramaya devam eder.
 * - Hazne dolduğunda (Feeder Full) durur.
 */
public class AutoIntakeCommand extends Command {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final VisionSubsystem vision;

    private final PIDController turnPID = new PIDController(2.0, 0.0, 0.0); // Tuning gerekebilir
    private final Timer lostTargetTimer = new Timer();
    private boolean hasSeenTarget = false;

    public AutoIntakeCommand(DriveSubsystem drive, IntakeSubsystem intake, 
                             FeederSubsystem feeder, VisionSubsystem vision) {
        this.drive = drive;
        this.intake = intake;
        this.feeder = feeder;
        this.vision = vision;
        
        addRequirements(drive, intake, feeder); // Vision requirement opsiyonel
    }

    @Override
    public void initialize() {
        // Pipeline'ı Game Piece moduna al
        vision.setPipeline(VisionConstants.kGamePiecePipelineIndex);
        
        // Timer ve durum sıfırla
        lostTargetTimer.stop();
        lostTargetTimer.reset();
        hasSeenTarget = false;
        
        // PID Reset
        turnPID.reset();
    }

    @Override
    public void execute() {
        boolean seesGamePiece = intake.seesGamePiece() || vision.hasGamePiece();
        double rotationSpeed = 0.0;
        double forwardSpeed = 0.0;

        if (seesGamePiece) {
            // Hedef görüldü!
            hasSeenTarget = true;
            lostTargetTimer.stop();
            lostTargetTimer.reset();

            // 1. Hizalanma (PID)
            // Hata: Intake Tx veya Vision Yaw
            double error = intake.seesGamePiece() ? intake.getAlignmentError() : vision.getGamePieceYaw();
            // Tx derece cinsinden, PID'ye verilebilir. 
            // Radyan'a çevirmek daha doğru olabilir ama PID katsayısı dereceye göre de ayarlanabilir.
            // Limelight tx +/- 29.8 derece.
            // PID output rad/s olmalı (Drive runVelocity için).
            // Derece -> Output (Rad/S) dönüşümü için kP'yi ona göre seçtik.
            // Basitçe: kP * errorDegrees. kP=0.1 dersek, 10 derece hata -> 1 rad/s dönüş.        
            rotationSpeed = turnPID.calculate(error, 0.0);
            
            // 2. İleri Sürüş (Yavaşça yaklaş)
            forwardSpeed = 1.0; // m/s (Ayarlanabilir)

        } else {
            // Hedef yok
            if (hasSeenTarget) {
                // Önceden görmüştük, kaybettik -> Timeout say
                lostTargetTimer.start();
                
                if (lostTargetTimer.get() < IntakeConstants.kTargetLostTimeoutSeconds) {
                    // Kayıp ama süre bitmedi -> Aramaya devam et veya düz git
                    // '3a bir şey görmüyor ise... 4 saniye intake çalışsın'
                    forwardSpeed = 0.5; // Yavaşça ilerle
                    rotationSpeed = 0.0;
                } else {
                    // Timeout doldu -> Dur
                    forwardSpeed = 0.0;
                    rotationSpeed = 0.0;
                }
            } else {
                // Hiç görmedik -> Olduğun yerde dur veya ara (şimdilik dur)
                forwardSpeed = 0.0;
                rotationSpeed = 0.0;
            }
        }
        
        // 3. Drive Uygula
        drive.runVelocity(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed));

        // 4. Intake ve Feeder Çalıştır
        // Eğer target görüyor veya timeout içindeysek çalıştır
        if (seesGamePiece || (hasSeenTarget && lostTargetTimer.get() < IntakeConstants.kTargetLostTimeoutSeconds)) {
             intake.runRoller(10.0); // 10 Volt (veya kAutoIntakeSpeed)
             feeder.feed(); // Feeder ileri
        } else {
             intake.runRoller(0.0);
             feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Durdur
        drive.stop();
        intake.runRoller(0.0);
        feeder.stop();
        
        // Pipeline'ı AprilTag'e geri al
        vision.setPipeline(VisionConstants.kAprilTagPipelineIndex);
    }

    @Override
    public boolean isFinished() {
        // 1. Tank Doldu mu?
        if (feeder.isFuelSystemFull()) {
            return true;
        }
        
        // 2. Timeout doldu mu? (Hedef görülüp kaybolduysa)
        if (hasSeenTarget && lostTargetTimer.hasElapsed(IntakeConstants.kTargetLostTimeoutSeconds)) {
            return true;
        }
        
        return false;
    }
}
