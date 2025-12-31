package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GamePieceConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Otomatik intake komutu.
 * 
 * <p>Robot kameradaki hedefe PID ile kilitlenir, yaklaşır ve intake çalıştırır.
 * TOF sensörü nesneyi algıladığında komut biter ve kumanda titrer.</p>
 */
public class AutoIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController controller;

    // PID Controller for rotation alignment
    private final PIDController alignPID;

    // State tracking
    private boolean hasCollected = false;

    /**
     * AutoIntakeCommand oluşturur.
     * 
     * @param intakeSubsystem Intake alt sistemi
     * @param driveSubsystem Sürüş alt sistemi
     * @param controller Titreşim için controller
     */
    public AutoIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            DriveSubsystem driveSubsystem,
            CommandXboxController controller) {

        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;

        // PID ayarları
        this.alignPID = new PIDController(
            GamePieceConstants.kAlignP,
            GamePieceConstants.kAlignI,
            GamePieceConstants.kAlignD
        );
        alignPID.setTolerance(GamePieceConstants.kAlignTolerance);
        alignPID.enableContinuousInput(-180, 180);

        addRequirements(intakeSubsystem, driveSubsystem);
    }

    @Override
    public void initialize() {
        hasCollected = false;
        alignPID.reset();
        
        // Intake'i başlat
        intakeSubsystem.runRoller(intakeSubsystem.getActiveIntakeSpeed());
    }

    @Override
    public void execute() {
        double rotationSpeed = 0.0;
        double forwardSpeed = 0.0;

        // Eğer aktif oyun nesnesini görüyorsak
        if (intakeSubsystem.seesActiveGamePiece()) {
            // 1. Yatay hizalama (PID ile dönüş)
            double tx = intakeSubsystem.getTargetTx();
            rotationSpeed = alignPID.calculate(tx, 0.0);

            // 2. İleri hareket (hedef alanına göre)
            double targetArea = intakeSubsystem.getTargetArea();
            if (targetArea > GamePieceConstants.kMinTargetArea) {
                // Hedefe yaklaşırken yavaşla
                forwardSpeed = GamePieceConstants.kApproachSpeed * (1.0 - Math.min(targetArea / 10.0, 0.5));
            }
        }

        // Robotu hareket ettir
        driveSubsystem.runVelocity(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed));

        // Intake motorunu çalıştır
        intakeSubsystem.runRoller(intakeSubsystem.getActiveIntakeSpeed());

        // TOF sensörü kontrol et
        if (intakeSubsystem.hasGamePieceInIntake()) {
            hasCollected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Motorları durdur
        driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
        intakeSubsystem.runRoller(0);

        // Başarılı toplama ise kumandayı titret
        if (hasCollected && !interrupted) {
            rumbleController();
        }
    }

    @Override
    public boolean isFinished() {
        return hasCollected;
    }

    /**
     * Controller'ı titretir (başarı geri bildirimi).
     */
    private void rumbleController() {
        // 0.5 saniye titreşim
        controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        
        // Titreşimi durdurmak için bir süre sonra sıfırla
        // Bu normalde bir Command ile yapılır ama basitlik için burada bırakıyoruz
        new Thread(() -> {
            try {
                Thread.sleep(500);
                controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }
}
