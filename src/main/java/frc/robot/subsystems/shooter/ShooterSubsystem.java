package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Shooter alt sistemi.
 * Turret (yatay) + Hood (dikey açı) + Flywheel kontrolü.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Supplier<Pose2d> robotPoseSupplier;
    private IntakeSubsystem intakeSubsystem;

    // Hedefin Sahadaki Konumu
    private final Translation2d targetLocation = new Translation2d(16.5, 5.55);

    // Turret PID
    private final PIDController turretPID = new PIDController(
            ShooterConstants.kTurretP,
            ShooterConstants.kTurretI,
            ShooterConstants.kTurretD);

    // Flywheel
    private double targetFlywheelSpeed = 0.0;

    // Hood (Atış Açısı)
    private double targetHoodAngle = ShooterConstants.kHoodMidAngle;

    // Edge detection
    private boolean lastShooterSensorState = false;

    // Tunable PID
    private final TunableNumber tunableHoodP;
    private final TunableNumber tunableHoodI;
    private final TunableNumber tunableHoodD;

    public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;

        turretPID.enableContinuousInput(-Math.PI, Math.PI);
        turretPID.setTolerance(ShooterConstants.kTurretTolerance);

        // Tunable Hood PID
        tunableHoodP = new TunableNumber("Shooter", "Hood P", ShooterConstants.kHoodP);
        tunableHoodI = new TunableNumber("Shooter", "Hood I", ShooterConstants.kHoodI);
        tunableHoodD = new TunableNumber("Shooter", "Hood D", ShooterConstants.kHoodD);
    }

    public void setIntakeSubsystem(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Edge detection for shooter sensor
        if (inputs.shooterSensorTriggered && !lastShooterSensorState) {
            if (intakeSubsystem != null) {
                intakeSubsystem.decrementItemCount();
            }
        }
        lastShooterSensorState = inputs.shooterSensorTriggered;

        // --- TURRET AUTO-AIM ---
        Pose2d robotPose = robotPoseSupplier.get();
        double targetAngleRad = Math.atan2(
                targetLocation.getY() - robotPose.getY(),
                targetLocation.getX() - robotPose.getX());
        Rotation2d robotHeading = robotPose.getRotation();
        double turretTargetAngle = MathUtil.angleModulus(targetAngleRad - robotHeading.getRadians());
        double turretVolts = turretPID.calculate(inputs.turretAbsolutePositionRad, turretTargetAngle);
        io.setTurretVoltage(turretVolts);

        // --- HOOD CONTROL ---
        io.setHoodAngle(targetHoodAngle);

        // --- FLYWHEEL CONTROL ---
        if (targetFlywheelSpeed > 0) {
            io.setFlywheelVoltage(ShooterConstants.kFlywheelVoltage);
        } else {
            io.setFlywheelVoltage(0.0);
        }

        // Loglama
        Logger.recordOutput("Shooter/TargetPose", targetLocation);
        Logger.recordOutput("Shooter/TurretGoal", turretTargetAngle);
        Logger.recordOutput("Shooter/TurretActual", inputs.turretAbsolutePositionRad);
        Logger.recordOutput("Shooter/HoodTarget", targetHoodAngle);
        Logger.recordOutput("Shooter/HoodActual", inputs.hoodPositionDegrees);
    }

    // ==================== COMMANDS ====================

    public void shoot() {
        targetFlywheelSpeed = ShooterConstants.kFlywheelTargetRPM;
    }

    public void stopShooter() {
        targetFlywheelSpeed = 0;
    }

    /** Hood açısını ayarla (derece) */
    public void setHoodAngle(double angleDegrees) {
        targetHoodAngle = MathUtil.clamp(
                angleDegrees,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
    }

    /** Yakın atış için hood ayarı */
    public void setHoodClose() {
        setHoodAngle(ShooterConstants.kHoodCloseAngle);
    }

    /** Orta mesafe için hood ayarı */
    public void setHoodMid() {
        setHoodAngle(ShooterConstants.kHoodMidAngle);
    }

    /** Uzak atış için hood ayarı */
    public void setHoodFar() {
        setHoodAngle(ShooterConstants.kHoodFarAngle);
    }

    // ==================== STATUS ====================

    public boolean isAimingAtTarget() {
        return turretPID.atSetpoint();
    }

    public boolean isHoodAtTarget() {
        return Math.abs(inputs.hoodPositionDegrees - targetHoodAngle) < ShooterConstants.kHoodTolerance;
    }

    public boolean isReadyToShoot() {
        return isAimingAtTarget() && isHoodAtTarget();
    }

    public double getHoodAngle() {
        return inputs.hoodPositionDegrees;
    }
}