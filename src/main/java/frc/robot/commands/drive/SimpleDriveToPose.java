package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Basit PID tabanlı hedefe gitme komutu.
 * Dönüş ve hareket eş zamanlı yapılır — robot yolda döner.
 * Hedefe varış toleransları geniş tutularak hızlı geçiş sağlanır.
 */
public class SimpleDriveToPose extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;

    // PID Kontrolcüleri
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    // Hız limitleri (m/s ve rad/s)
    private static final double MAX_LINEAR_SPEED = 4.5; // m/s (max 4.8)
    private static final double MAX_ROT_SPEED = 3.5;    // rad/s

    // Toleranslar — geniş tutuldu, hızlı geçiş için
    private static final double POSITION_TOLERANCE = 0.15; // metre
    private static final double ROTATION_TOLERANCE = 0.1;  // radyan (~6 derece)

    public SimpleDriveToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;

        // Yüksek P kazancı → hızlı yanıt, D → sönümleme (overshoot önler)
        xController = new PIDController(4.0, 0.0, 0.2);
        yController = new PIDController(4.0, 0.0, 0.2);
        rotController = new PIDController(4.5, 0.0, 0.2);

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotController.setTolerance(ROTATION_TOLERANCE);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotController.reset();
        System.out.println("[SimpleDrive] New Target: " + targetPose);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        // PID çıkışları
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Hız limitleri
        xSpeed = MathUtil.clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        ySpeed = MathUtil.clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        rotSpeed = MathUtil.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // Log
        org.littletonrobotics.junction.Logger.recordOutput("SimpleDrive/Target", targetPose);

        // Saha-referanslı sürüş (eş zamanlı hareket + dönüş)
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, -rotSpeed, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
        System.out.println("[SimpleDrive] Finished " + (interrupted ? "(Interrupted)" : ""));
    }

    @Override
    public boolean isFinished() {
        // Sadece pozisyon yeterli olduğunda bitir
        // Dönüş yolda tamamlanır, son noktada mükemmel olması gerekmez
        return xController.atSetpoint() && yController.atSetpoint();
    }
}