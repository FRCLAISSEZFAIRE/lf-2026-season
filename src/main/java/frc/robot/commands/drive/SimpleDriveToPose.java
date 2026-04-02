package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Basit PID tabanlı hedefe gitme komutu.
 * PathPlanner bağımsız çalışır.
 */
public class SimpleDriveToPose extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;

    // PID Kontrolcüleri
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    // Tunable Toleranslar
    private static final TunableNumber posTolerance = new TunableNumber("SimpleDrive", "PositionTolerance", 0.2);
    private static final TunableNumber rotTolerance = new TunableNumber("SimpleDrive", "RotationTolerance", 0.08);

    // Tunable PID Değerleri
    private static final TunableNumber kPX = new TunableNumber("SimpleDrive", "kP_X", 2.0);
    private static final TunableNumber kIX = new TunableNumber("SimpleDrive", "kI_X", 0.0);
    private static final TunableNumber kDX = new TunableNumber("SimpleDrive", "kD_X", 0.1);

    private static final TunableNumber kPY = new TunableNumber("SimpleDrive", "kP_Y", 2.0);
    private static final TunableNumber kIY = new TunableNumber("SimpleDrive", "kI_Y", 0.0);
    private static final TunableNumber kDY = new TunableNumber("SimpleDrive", "kD_Y", 0.1);

    private static final TunableNumber kPRot = new TunableNumber("SimpleDrive", "kP_Rot", 3.0);
    private static final TunableNumber kIRot = new TunableNumber("SimpleDrive", "kI_Rot", 0.0);
    private static final TunableNumber kDRot = new TunableNumber("SimpleDrive", "kD_Rot", 0.1);

    public SimpleDriveToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;

        // PID ayarları
        xController = new PIDController(kPX.get(), kIX.get(), kDX.get());
        yController = new PIDController(kPY.get(), kIY.get(), kDY.get());
        rotController = new PIDController(kPRot.get(), kIRot.get(), kDRot.get());

        // Rotation controller için continuous input
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Toleranslar
        xController.setTolerance(posTolerance.get());
        yController.setTolerance(posTolerance.get());
        rotController.setTolerance(rotTolerance.get());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (FieldConstants.isPoseForbidden(targetPose)) {
            System.err.println("[SimpleDrive] HATA: Hedef pozisyon YASAKLI BÖLGEDE! " + targetPose);
        }

        System.out.println("[SimpleDrive] Başlatıldı. Hedef: " + targetPose);

        // PID'leri sıfırla
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        // PID Update
        if (kPX.hasChanged() || kIX.hasChanged() || kDX.hasChanged()) {
            xController.setPID(kPX.get(), kIX.get(), kDX.get());
        }
        if (kPY.hasChanged() || kIY.hasChanged() || kDY.hasChanged()) {
            yController.setPID(kPY.get(), kIY.get(), kDY.get());
        }
        if (kPRot.hasChanged() || kIRot.hasChanged() || kDRot.hasChanged()) {
            rotController.setPID(kPRot.get(), kIRot.get(), kDRot.get());
        }

        // Tolerance Update
        if (posTolerance.hasChanged()) {
            xController.setTolerance(posTolerance.get());
            yController.setTolerance(posTolerance.get());
        }
        if (rotTolerance.hasChanged()) {
            rotController.setTolerance(rotTolerance.get());
        }

        // Hata hesapla
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Hız limitleri
        xSpeed = clamp(xSpeed, -3.0, 3.0);
        ySpeed = clamp(ySpeed, -3.0, 3.0);
        rotSpeed = clamp(rotSpeed, -2.0, 2.0);

        // Log: Hedef poz
        org.littletonrobotics.junction.Logger.recordOutput("SimpleDrive/Target", targetPose);

        // Saha-referanslı sürüş
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, -rotSpeed, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}