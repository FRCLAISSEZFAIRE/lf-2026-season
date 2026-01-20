package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Shooter alt sistemi.
 * Turret (yatay) + Hood (dikey açı) + Flywheel kontrolü.
 * 
 * <h2>Turret Özellikleri:</h2>
 * <ul>
 * <li>NEO motor + Harici REV Absolute Encoder</li>
 * <li>Seeding: Başlangıçta absolute encoder'dan pozisyon okuma</li>
 * <li>Continuous Wrapping: -180° ile 180° arası en kısa yol</li>
 * <li>Soft Limits: Yazılımsal güvenlik limitleri</li>
 * <li>TunableNumber: Runtime PID tuning</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterMode {
        IDLE,
        SCORING,
        FEEDING
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Supplier<Pose2d> robotPoseSupplier;
    private IntakeSubsystem intakeSubsystem;

    // Target değerleri
    private double targetFlywheelSpeed = ShooterConstants.kIdleFlywheelRPM;
    private double targetHoodAngle = ShooterConstants.kHoodMidAngle;
    private double targetTurretAngle = 0.0; // Derece

    // Mod ve durum
    private ShooterMode currentMode = ShooterMode.IDLE;
    private boolean isAutoAimActive = false;
    private double distanceToTarget = 0.0;
    private boolean isInAllianceZone = false;

    // Edge detection
    private boolean lastShooterSensorState = false;

    // ===========================================================================
    // TUNABLE PID - AdvantageScope üzerinden runtime'da değiştirilebilir
    // ===========================================================================
    private final TunableNumber tunableTurretP;
    private final TunableNumber tunableTurretI;
    private final TunableNumber tunableTurretD;

    public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;

        // TunableNumber PID oluştur
        tunableTurretP = new TunableNumber("Shooter", "Turret P", ShooterConstants.kTurretDefaultP);
        tunableTurretI = new TunableNumber("Shooter", "Turret I", ShooterConstants.kTurretDefaultI);
        tunableTurretD = new TunableNumber("Shooter", "Turret D", ShooterConstants.kTurretDefaultD);
    }

    public void setIntakeSubsystem(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    // ===========================================================================
    // ALLIANCE-AWARE TARGETING
    // ===========================================================================

    public boolean isInAllianceZone() {
        Pose2d robotPose = robotPoseSupplier.get();
        double robotX = robotPose.getX();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return robotX >= FieldConstants.kRedAllianceZoneMinX;
        }
        return robotX <= FieldConstants.kBlueAllianceZoneMaxX;
    }

    public Translation2d getCurrentTarget() {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        if (isInAllianceZone) {
            return isRed ? FieldConstants.kRedHubPose.getTranslation()
                    : FieldConstants.kBlueHubPose.getTranslation();
        } else {
            return isRed ? FieldConstants.kRedFeedingTarget
                    : FieldConstants.kBlueFeedingTarget;
        }
    }

    public Translation2d getHubLocation() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.kRedHubPose.getTranslation();
        }
        return FieldConstants.kBlueHubPose.getTranslation();
    }

    public double getDistanceToTarget() {
        Pose2d robotPose = robotPoseSupplier.get();
        return robotPose.getTranslation().getDistance(getCurrentTarget());
    }

    // ===========================================================================
    // PERIODIC
    // ===========================================================================

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // --- TUNABLE PID CHECK ---
        // PID değerleri değiştiyse motor sürücüyü güncelle
        if (tunableTurretP.hasChanged() || tunableTurretI.hasChanged() || tunableTurretD.hasChanged()) {
            if (io instanceof ShooterIOReal) {
                ((ShooterIOReal) io).configureTurret(
                        tunableTurretP.get(),
                        tunableTurretI.get(),
                        tunableTurretD.get());
                System.out.println("[Turret] PID Updated: P=" + tunableTurretP.get() +
                        " I=" + tunableTurretI.get() +
                        " D=" + tunableTurretD.get());
            }
        }

        // Edge detection for shooter sensor
        if (inputs.shooterSensorTriggered && !lastShooterSensorState) {
            if (intakeSubsystem != null) {
                intakeSubsystem.decrementItemCount();
            }
        }
        lastShooterSensorState = inputs.shooterSensorTriggered;

        // Zone ve mesafe hesapla
        isInAllianceZone = isInAllianceZone();
        Translation2d currentTarget = getCurrentTarget();
        distanceToTarget = getDistanceToTarget();

        // Mod belirle
        if (isAutoAimActive) {
            currentMode = isInAllianceZone ? ShooterMode.SCORING : ShooterMode.FEEDING;
        } else {
            currentMode = ShooterMode.IDLE;
        }

        // --- AUTO-AIM HOOD & FLYWHEEL ---
        if (isAutoAimActive) {
            if (currentMode == ShooterMode.SCORING) {
                targetHoodAngle = ShooterConstants.getHoodAngleForDistance(distanceToTarget);
                targetFlywheelSpeed = ShooterConstants.getFlywheelRPMForDistance(distanceToTarget);
            } else {
                targetHoodAngle = ShooterConstants.kFeedingHoodAngle;
                targetFlywheelSpeed = ShooterConstants.kFeedingFlywheelRPM;
            }
        }

        // --- TURRET AUTO-AIM ---
        if (ShooterConstants.kIsTurreted) {
            Pose2d robotPose = robotPoseSupplier.get();
            double targetAngleRad = Math.atan2(
                    currentTarget.getY() - robotPose.getY(),
                    currentTarget.getX() - robotPose.getX());
            Rotation2d robotHeading = robotPose.getRotation();
            // Robot heading'i çıkar -> turret'in robot gövdesine göre açısı
            double turretAngleRad = MathUtil.angleModulus(targetAngleRad - robotHeading.getRadians());
            targetTurretAngle = Math.toDegrees(turretAngleRad);

            // SparkMax closed-loop position control (radyan olarak gönder - IO içinde
            // dereceye çevrilir)
            io.setTurretPosition(turretAngleRad);
        }

        // --- HOOD CONTROL ---
        double clampedHoodAngle = MathUtil.clamp(
                targetHoodAngle,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
        io.setHoodAngle(clampedHoodAngle);

        // --- FLYWHEEL CONTROL ---
        io.setFlywheelVelocity(targetFlywheelSpeed);
        if (ShooterConstants.kHasDualFlywheels) {
            io.setFlywheelRightVelocity(targetFlywheelSpeed);
        }

        // --- LOGGING ---
        Logger.recordOutput("Shooter/Mode", currentMode.toString());
        Logger.recordOutput("Shooter/InAllianceZone", isInAllianceZone);
        Logger.recordOutput("Shooter/CurrentTarget", currentTarget);
        Logger.recordOutput("Shooter/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Shooter/TurretGoalDeg", targetTurretAngle);
        Logger.recordOutput("Shooter/TurretActualDeg", Math.toDegrees(inputs.turretAbsolutePositionRad));
        Logger.recordOutput("Shooter/HoodTarget", clampedHoodAngle);
        Logger.recordOutput("Shooter/HoodActual", inputs.hoodPositionDegrees);
        Logger.recordOutput("Shooter/FlywheelTargetRPM", targetFlywheelSpeed);
        Logger.recordOutput("Shooter/FlywheelActualRPM", inputs.flywheelVelocityRadPerSec * 60.0 / (2 * Math.PI));
        Logger.recordOutput("Shooter/AutoAimActive", isAutoAimActive);
        Logger.recordOutput("Shooter/IsReadyToShoot", isReadyToShoot());

        // Tunable PID log
        Logger.recordOutput("Shooter/TunableTurretP", tunableTurretP.get());
        Logger.recordOutput("Shooter/TunableTurretI", tunableTurretI.get());
        Logger.recordOutput("Shooter/TunableTurretD", tunableTurretD.get());
    }

    // ===========================================================================
    // COMMANDS
    // ===========================================================================

    public void enableAutoAim() {
        isAutoAimActive = true;
    }

    public void disableAutoAim() {
        isAutoAimActive = false;
        targetFlywheelSpeed = ShooterConstants.kIdleFlywheelRPM;
        currentMode = ShooterMode.IDLE;
    }

    public void shoot() {
        enableAutoAim();
    }

    public void stopShooter() {
        disableAutoAim();
    }

    public void stopMotorTotal() {
        isAutoAimActive = false;
        targetFlywheelSpeed = 0.0;
        currentMode = ShooterMode.IDLE;
    }

    public void reverse() {
        isAutoAimActive = false;
        targetFlywheelSpeed = -1000.0;
    }

    // ===========================================================================
    // MANUEL HOOD KONTROL
    // ===========================================================================

    public void setHoodAngle(double angleDegrees) {
        if (!isAutoAimActive) {
            targetHoodAngle = MathUtil.clamp(angleDegrees, ShooterConstants.kHoodMinAngle,
                    ShooterConstants.kHoodMaxAngle);
        }
    }

    public void setHoodClose() {
        setHoodAngle(ShooterConstants.kHoodCloseAngle);
    }

    public void setHoodMid() {
        setHoodAngle(ShooterConstants.kHoodMidAngle);
    }

    public void setHoodFar() {
        setHoodAngle(ShooterConstants.kHoodFarAngle);
    }

    // ===========================================================================
    // STATUS
    // ===========================================================================

    public ShooterMode getCurrentMode() {
        return currentMode;
    }

    public boolean isAutoAimActive() {
        return isAutoAimActive;
    }

    public boolean isAimingAtTarget() {
        if (ShooterConstants.kIsTurreted) {
            double turretActualDeg = Math.toDegrees(inputs.turretAbsolutePositionRad);
            double turretError = MathUtil.angleModulus(Math.toRadians(targetTurretAngle - turretActualDeg));
            return Math.abs(Math.toDegrees(turretError)) < ShooterConstants.kTurretTolerance;
        } else {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d target = getCurrentTarget();
            double targetAngleRad = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX());
            double errorRad = MathUtil.angleModulus(targetAngleRad - robotPose.getRotation().getRadians());
            return Math.abs(Math.toDegrees(errorRad)) < ShooterConstants.kTurretTolerance;
        }
    }

    public boolean isHoodAtTarget() {
        return Math.abs(inputs.hoodPositionDegrees - targetHoodAngle) < ShooterConstants.kHoodTolerance;
    }

    public boolean isFlywheelAtSpeed() {
        double currentRPM = inputs.flywheelVelocityRadPerSec * 60.0 / (2 * Math.PI);
        return Math.abs(currentRPM - targetFlywheelSpeed) < ShooterConstants.kFlywheelToleranceRPM;
    }

    public boolean isReadyToShoot() {
        return isAutoAimActive && isAimingAtTarget() && isHoodAtTarget() && isFlywheelAtSpeed();
    }

    public boolean isInShootingRange() {
        return isInAllianceZone && distanceToTarget >= ShooterConstants.kMinShootingDistance
                && distanceToTarget <= ShooterConstants.kMaxShootingDistance;
    }

    // ===========================================================================
    // GETTERS
    // ===========================================================================

    public double getHoodAngle() {
        return inputs.hoodPositionDegrees;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getDistanceToTargetCached() {
        return distanceToTarget;
    }

    public double getTargetFlywheelSpeed() {
        return targetFlywheelSpeed;
    }

    public boolean getIsInAllianceZone() {
        return isInAllianceZone;
    }

    public double getTargetTurretAngle() {
        return targetTurretAngle;
    }
}