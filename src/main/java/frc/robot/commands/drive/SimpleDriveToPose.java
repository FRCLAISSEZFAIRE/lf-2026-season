package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

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

    // Toleranslar
    private static final double POSITION_TOLERANCE = 0.1; // metre
    private static final double ROTATION_TOLERANCE = 0.05; // radyan (~3 derece)

    public SimpleDriveToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;

        // PID ayarları
        xController = new PIDController(2.0, 0.0, 0.1);
        yController = new PIDController(2.0, 0.0, 0.1);
        rotController = new PIDController(3.0, 0.0, 0.1);

        // Rotation controller için continuous input
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Toleranslar
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotController.setTolerance(ROTATION_TOLERANCE);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // PID'leri sıfırla
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

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
        // xSpeed ve ySpeed ters çevrildi: motor/encoder ayarı nedeniyle pozitif vx =
        // fiziksel geriye gidiş.
        // DriveWithJoystick'teki Drive/InvertJoystick ile aynı telafi.
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