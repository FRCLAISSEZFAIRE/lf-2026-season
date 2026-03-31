package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");

    public enum ShooterMode {
        IDLE,
        SCORING,
        FEEDING
    }

    private final Supplier<Pose2d> robotPoseSupplier;
    private ShooterMode currentMode = ShooterMode.IDLE;
    private boolean isAutoAimActive = false;

    private double turretTargetDeg = 0;
    private double hoodTargetDeg = ShooterConstants.kHoodMidAngle;
    private double flywheelTargetRPM = ShooterConstants.kIdleFlywheelRPM;
    private double autoAimOffsetDeg = 0.0;
    private double hubOffsetX = 0.0;
    private double hubOffsetY = 0.0;
    private double flywheelOffsetRPM = 0.0;
    private double hoodOffsetDeg = 0.0;

    private double lastFlywheelTargetRPM = 0;
    private double lastAutoAimOffsetDeg = 0;
    private double lastHubOffsetX = 0;
    private double lastHubOffsetY = 0;
    private double lastFlywheelOffsetRPM = 0;
    private double lastHoodOffsetDeg = 0;

    private final InterpolatingDoubleTreeMap hoodCalibrationMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelCalibrationMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAlliancePassMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelAlliancePassMap = new InterpolatingDoubleTreeMap();

    private final TunableNumber turretKP = new TunableNumber("Shooter", "Turret kP",
            ShooterConstants.kTurretDefaultP);
    private final TunableNumber turretKI = new TunableNumber("Shooter", "Turret kI",
            ShooterConstants.kTurretDefaultI);
    private final TunableNumber turretKD = new TunableNumber("Shooter", "Turret kD",
            ShooterConstants.kTurretDefaultD);

    private final TunableNumber hoodKP = new TunableNumber("Shooter", "Hood kP", ShooterConstants.kHoodP);
    private final TunableNumber hoodKI = new TunableNumber("Shooter", "Hood kI", ShooterConstants.kHoodI);
    private final TunableNumber hoodKD = new TunableNumber("Shooter", "Hood kD", ShooterConstants.kHoodD);

    private final TunableNumber flywheelKP = new TunableNumber("Shooter", "Flywheel kP",
            ShooterConstants.kFlywheelP);
    private final TunableNumber flywheelKI = new TunableNumber("Shooter", "Flywheel kI",
            ShooterConstants.kFlywheelI);
    private final TunableNumber flywheelKD = new TunableNumber("Shooter", "Flywheel kD",
            ShooterConstants.kFlywheelD);
    private final TunableNumber flywheelKV = new TunableNumber("Shooter", "Flywheel kV",
            ShooterConstants.kFlywheelkV);
    private final TunableNumber flywheelTolerance = new TunableNumber("Shooter", "Flywheel Tolerance",
            ShooterConstants.SHOOTER_RPM_TOLERANCE);
    private final TunableNumber turretTolerance = new TunableNumber("Shooter", "Turret Tolerance",
            ShooterConstants.TURRET_AIM_TOLERANCE);
    private final TunableNumber hoodTolerance = new TunableNumber("Shooter", "Hood Tolerance",
            2.0); // Default to 2.0 deg

    private final TunableNumber turretMaxOutput = new TunableNumber("Shooter", "Turret MaxOutput",
            ShooterConstants.kTurretMaxOutput);
    private final TunableNumber turretCenterOffset = new TunableNumber("Shooter", "Turret Center", 0.0);

    private final TunableNumber turretMinAngle = new TunableNumber("Shooter", "Turret MinAngle",
            ShooterConstants.kTurretMinAngle);
    private final TunableNumber turretMaxAngle = new TunableNumber("Shooter", "Turret MaxAngle",
            ShooterConstants.kTurretMaxAngle);

    private final TunableNumber turretGearRatio = new TunableNumber("Shooter", "Turret Gear Ratio",
            ShooterConstants.kTurretGearRatio);

    private final TunableNumber hoodGearRatio = new TunableNumber("Shooter", "Hood Gear Ratio",
            ShooterConstants.kHoodGearRatio);

    private final TunableNumber turretPositionX = new TunableNumber("Shooter", "Turret Center X", 0.0);
    private final TunableNumber turretPositionY = new TunableNumber("Shooter", "Turret Center Y", 0.0);

    private final frc.robot.util.TunableBoolean enableMovingShoot = new frc.robot.util.TunableBoolean(
            "Shooter", "Enable Moving Shoot", false);
    private final TunableNumber averageShotVelocityMps = new TunableNumber("Shooter",
            "Average Shot Velocity m/s", 18.0);

    private boolean lastFlywheelInvertState = false;

    public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/InvertFlywheel", false);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/EnableAutoAim", false);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/InvertAiming", false);

        initializeCalibration();
        System.out.println("[Shooter] AdvantageKit IO Başlatıldı");
    }

    private void createShooterMaps() {
        hoodCalibrationMap.clear();
        flywheelCalibrationMap.clear();
        hoodAlliancePassMap.clear();
        flywheelAlliancePassMap.clear();

        for (int i = 0; i < ShooterConstants.HUB_DIST_TUNABLES.length; i++) {
            double dist = ShooterConstants.HUB_DIST_TUNABLES[i].get();
            double rpm = ShooterConstants.HUB_RPM_TUNABLES[i].get();
            double hood = ShooterConstants.HUB_HOOD_TUNABLES[i].get();

            hoodCalibrationMap.put(dist, hood);
            flywheelCalibrationMap.put(dist, rpm);
        }

        for (int i = 0; i < ShooterConstants.PASS_DIST_TUNABLES.length; i++) {
            double dist = ShooterConstants.PASS_DIST_TUNABLES[i].get();
            double rpm = ShooterConstants.PASS_RPM_TUNABLES[i].get();
            double hood = ShooterConstants.PASS_HOOD_TUNABLES[i].get();

            hoodAlliancePassMap.put(dist, hood);
            flywheelAlliancePassMap.put(dist, rpm);
        }
    }

    private void initializeCalibration() {
        createShooterMaps();
    }

    private void updateCalibrationMapsFromDashboard() {
        boolean hasChanged = false;

        for (TunableNumber t : ShooterConstants.HUB_DIST_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.HUB_RPM_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.HUB_HOOD_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.PASS_DIST_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.PASS_RPM_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.PASS_HOOD_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;

        if (hasChanged) {
            createShooterMaps();
        }
    }

    public static class ShooterState {
        public final double rpm;
        public final double hoodAngleDeg;

        public ShooterState(double rpm, double hoodAngleDeg) {
            this.rpm = rpm;
            this.hoodAngleDeg = hoodAngleDeg;
        }
    }

    public ShooterState calculateShooterState(double distanceMeters) {
        if (Double.isNaN(distanceMeters) || distanceMeters <= 0) {
            return new ShooterState(ShooterConstants.FENDER_SHOT_RPM, ShooterConstants.FENDER_SHOT_HOOD_ANGLE);
        }

        double minDistance = ShooterConstants.kHubDist0.get();
        double maxDistance = ShooterConstants.kHubDist4.get();
        double clampedDistance = MathUtil.clamp(distanceMeters, minDistance, maxDistance);

        double rpm = flywheelCalibrationMap.get(clampedDistance);
        double hoodAngle = hoodCalibrationMap.get(clampedDistance);

        Logger.recordOutput("Tuning/Shooter/Calculated/Distance", distanceMeters);
        Logger.recordOutput("Tuning/Shooter/Calculated/RPM", rpm);
        Logger.recordOutput("Tuning/Shooter/Calculated/HoodAngle", hoodAngle);

        return new ShooterState(rpm, hoodAngle);
    }

    public ShooterState calculateShooterStateForAlliancePass(double distanceMeters) {
        if (Double.isNaN(distanceMeters) || distanceMeters <= 0) {
            return new ShooterState(ShooterConstants.FENDER_SHOT_RPM, ShooterConstants.FENDER_SHOT_HOOD_ANGLE);
        }

        double minDistance = ShooterConstants.kPassDist0.get();
        double maxDistance = ShooterConstants.kPassDist4.get();
        double clampedDistance = MathUtil.clamp(distanceMeters, minDistance, maxDistance);
        double targetRPM = flywheelAlliancePassMap.get(clampedDistance);
        double targetHood = hoodAlliancePassMap.get(clampedDistance);

        return new ShooterState(targetRPM, targetHood);
    }

    public void applyShooterState(ShooterState state) {
        if (state == null)
            return;
        setFlywheelRPM(state.rpm + flywheelOffsetRPM);
        setHoodAngle(state.hoodAngleDeg + hoodOffsetDeg);
    }

    private void checkFlywheelInversion() {
        boolean invert = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Tuning/Shooter/InvertFlywheel",
                false);
        if (invert != lastFlywheelInvertState) {
            lastFlywheelInvertState = invert;
            io.setFlywheelInverted(invert);
        }
    }

    private void checkAndApplyTunables() {
        checkFlywheelInversion();

        if (turretKP.hasChanged() || turretKI.hasChanged() || turretKD.hasChanged() || turretMaxOutput.hasChanged()) {
            io.setTurretPID(turretKP.get(), turretKI.get(), turretKD.get(), turretMaxOutput.get());
            System.out.println("[Shooter] Turret PID updated: P=" + turretKP.get() + " I=" + turretKI.get() + " D="
                    + turretKD.get() + " Max=" + turretMaxOutput.get());
        }

        if (turretMinAngle.hasChanged() || turretMaxAngle.hasChanged()) {
            io.setTurretSoftLimits(turretMinAngle.get(), turretMaxAngle.get());
            System.out.println(
                    "[Shooter] Turret Soft Limits: Min=" + turretMinAngle.get() + " Max=" + turretMaxAngle.get());
        }

        if (hoodKP.hasChanged() || hoodKI.hasChanged() || hoodKD.hasChanged()) {
            io.setHoodPID(hoodKP.get(), hoodKI.get(), hoodKD.get());
            System.out.println(
                    "[Shooter] Hood PID updated: P=" + hoodKP.get() + " I=" + hoodKI.get() + " D=" + hoodKD.get());
        }

        if (flywheelKP.hasChanged() || flywheelKI.hasChanged() || flywheelKD.hasChanged() || flywheelKV.hasChanged()) {
            io.setFlywheelPID(flywheelKP.get(), flywheelKI.get(), flywheelKD.get(), flywheelKV.get());
            System.out.println("[Shooter] Flywheel PID updated: P=" + flywheelKP.get() + " I=" + flywheelKI.get()
                    + " D=" + flywheelKD.get() + " V=" + flywheelKV.get());
        }

        if (turretGearRatio.hasChanged()) {
            io.configTurretGearRatio(turretGearRatio.get());
            System.out.println("[Shooter] Turret Gear Ratio: " + turretGearRatio.get());
        }
        if (hoodGearRatio.hasChanged()) {
            io.configHoodGearRatio(hoodGearRatio.get());
            System.out.println("[Shooter] Hood Gear Ratio: " + hoodGearRatio.get());
        }
    }

    private final TunableNumber tunableTargetOverrideX = new TunableNumber("Tuning/Shooter/Target", "OverrideX", 0);
    private final TunableNumber tunableTargetOverrideY = new TunableNumber("Tuning/Shooter/Target", "OverrideY", 0);

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        boolean dashEnabled = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getBoolean("Tuning/Shooter/EnableAutoAim", false);
        if (dashEnabled != isAutoAimActive) {
            if (dashEnabled)
                enableAutoAim();
            else
                disableAutoAim();
        }

        // --- LIVE TUNING & PREFERENCES ---
        checkAndApplyTunables();
        updateCalibrationMapsFromDashboard();

        // --- CONSOLE FEEDBACK FOR OFFSET CHANGES ---
        if (Math.abs(autoAimOffsetDeg - lastAutoAimOffsetDeg) > 0.01) {
            System.out.println("[Shooter] Turret Offset changed to: " + autoAimOffsetDeg);
            lastAutoAimOffsetDeg = autoAimOffsetDeg;
        }
        if (Math.abs(hoodOffsetDeg - lastHoodOffsetDeg) > 0.01) {
            System.out.println("[Shooter] Hood Offset changed to: " + hoodOffsetDeg);
            lastHoodOffsetDeg = hoodOffsetDeg;
        }
        if (Math.abs(flywheelOffsetRPM - lastFlywheelOffsetRPM) > 1.0) {
            System.out.println("[Shooter] Flywheel Offset changed to: " + flywheelOffsetRPM);
            lastFlywheelOffsetRPM = flywheelOffsetRPM;
        }
        if (Math.abs(hubOffsetX - lastHubOffsetX) > 0.001 || Math.abs(hubOffsetY - lastHubOffsetY) > 0.001) {
            System.out.println("[Shooter] Hub Offset changed to: (" + hubOffsetX + ", " + hubOffsetY + ")");
            lastHubOffsetX = hubOffsetX;
            lastHubOffsetY = hubOffsetY;
        }

        // --- CONSOLE FEEDBACK FOR TARGET CHANGES ---
        if (Math.abs(flywheelTargetRPM - lastFlywheelTargetRPM) > 5.0) {
            System.out.println(
                    "[Shooter] Flywheel Target changed to: " + String.format("%.0f", flywheelTargetRPM) + " RPM");
            lastFlywheelTargetRPM = flywheelTargetRPM;
        }

        // --- CONSOLE FEEDBACK FOR TUNABLE CHANGES ---
        if (turretTolerance.hasChanged())
            System.out.println("[Shooter] Turret Tolerance changed to: " + turretTolerance.get());
        if (flywheelTolerance.hasChanged())
            System.out.println("[Shooter] Flywheel Tolerance changed to: " + flywheelTolerance.get());
        if (hoodTolerance.hasChanged())
            System.out.println("[Shooter] Hood Tolerance changed to: " + hoodTolerance.get());

        // --- BACKGROUND TARGET TRACKING ---
        // Only run if auto-aim is enabled AND no command is currently using the shooter
        if (isAutoAimActive && getCurrentCommand() == null) {
            trackTarget();
        }

        logTelemetry();

        tuningTable.getEntry("Shooter/Ready").setBoolean(isReadyToShoot());
        tuningTable.getEntry("Shooter/AimLocked").setBoolean(isTurretAtTarget());
        tuningTable.getEntry("Shooter/FlywheelReady").setBoolean(isFlywheelAtTarget());
        tuningTable.getEntry("Shooter/HoodReady").setBoolean(isHoodAtTarget());
    }

    public void updateAiming(Pose2d robotPose, edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds) {
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(DriverStation.getAlliance());
        hubLocation = hubLocation.plus(new Translation2d(hubOffsetX, hubOffsetY));

        double initialDistance = turretFieldPosition.getDistance(hubLocation);

        if (enableMovingShoot.get() && fieldSpeeds != null) {
            hubLocation = frc.robot.util.MovingShootUtil.getVirtualTarget(hubLocation, turretFieldPosition, fieldSpeeds,
                    averageShotVelocityMps.get(), initialDistance);
        }

        double distance = turretFieldPosition.getDistance(hubLocation);
        double angleToTargetRad = Math.atan2(hubLocation.getY() - turretFieldPosition.getY(),
                hubLocation.getX() - turretFieldPosition.getX());
        double robotHeadingRad = robotPose.getRotation().getRadians();
        double targetTurretRad = MathUtil.angleModulus(angleToTargetRad - robotHeadingRad);

        turretTargetDeg = Math.toDegrees(targetTurretRad) + autoAimOffsetDeg;

        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg, turretMinAngle.get(), turretMaxAngle.get());
        }

        ShooterState state = calculateShooterState(distance);
        flywheelTargetRPM = state.rpm;
        hoodTargetDeg = state.hoodAngleDeg;

        setTurretAngle(turretTargetDeg);
        applyShooterState(state);

        Logger.recordOutput("Tuning/Shooter/Aiming/Distance", distance);
        Logger.recordOutput("Tuning/Shooter/Aiming/TargetTurretAngle", turretTargetDeg);
        Logger.recordOutput("Tuning/Shooter/Aiming/TurretError", turretTargetDeg - getTurretAngle());
    }

    public void updateAiming(Pose2d robotPose) {
        updateAiming(robotPose, new edu.wpi.first.math.kinematics.ChassisSpeeds());
    }

    public void updateAimingManualRPM(double manualRPM) {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(DriverStation.getAlliance());
        hubLocation = hubLocation.plus(new Translation2d(hubOffsetX, hubOffsetY));

        double distance = turretFieldPosition.getDistance(hubLocation);
        double angleToTargetRad = Math.atan2(hubLocation.getY() - turretFieldPosition.getY(),
                hubLocation.getX() - turretFieldPosition.getX());
        double targetTurretRad = MathUtil.angleModulus(angleToTargetRad - robotPose.getRotation().getRadians());
        turretTargetDeg = Math.toDegrees(targetTurretRad) + autoAimOffsetDeg;

        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg, turretMinAngle.get(), turretMaxAngle.get());
        }

        ShooterState state = calculateShooterState(distance);
        hoodTargetDeg = state.hoodAngleDeg;
        flywheelTargetRPM = manualRPM;

        setTurretAngle(turretTargetDeg);
        setHoodAngle(hoodTargetDeg);
        setFlywheelRPM(flywheelTargetRPM);
    }

    public void updateAimingForPass(Pose2d robotPose) {
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));
        Translation2d targetLocation = FieldConstants.getPassTarget(DriverStation.getAlliance(), robotPose);
        double distance = turretFieldPosition.getDistance(targetLocation);
        double angleToTargetRad = Math.atan2(targetLocation.getY() - turretFieldPosition.getY(),
                targetLocation.getX() - turretFieldPosition.getX());
        double targetTurretRad = MathUtil.angleModulus(angleToTargetRad - robotPose.getRotation().getRadians());

        turretTargetDeg = Math.toDegrees(targetTurretRad) + autoAimOffsetDeg;

        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg, turretMinAngle.get(), turretMaxAngle.get());
        }

        ShooterState state = calculateShooterStateForAlliancePass(distance);
        setTurretAngle(turretTargetDeg);
        applyShooterState(state);
    }

    public boolean isReadyToShoot() {
        double rpmError = Math.abs(getFlywheelRPM() - flywheelTargetRPM);
        double turretError = Math.abs(getTurretAngle() - turretTargetDeg);
        double hoodError = Math.abs(getHoodAngle() - hoodTargetDeg);

        boolean rpmReady = rpmError < flywheelTolerance.get();
        boolean turretReady = turretError < turretTolerance.get();
        boolean hoodReady = hoodError < hoodTolerance.get();

        if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() % 1.0 < 0.02) { // Log every ~1s in console
            if (!rpmReady || !turretReady || !hoodReady) {
                System.out.println("[Shooter] Not Ready: " +
                        (rpmReady ? ""
                                : "RPM_Err=" + String.format("%.1f", rpmError) + " (Tol=" + flywheelTolerance.get()
                                        + ") ")
                        +
                        (turretReady ? ""
                                : "Turret_Err="
                                        + String.format("%.2f", turretError) + " (Tol=" + turretTolerance.get() + ") ")
                        +
                        (hoodReady ? ""
                                : "Hood_Err=" + String.format("%.2f", hoodError) + " (Tol=" + hoodTolerance.get()
                                        + ")"));
            }
        }

        Logger.recordOutput("Tuning/Shooter/Readiness/RPM_Error", rpmError);
        Logger.recordOutput("Tuning/Shooter/Readiness/Turret_Error", turretError);
        Logger.recordOutput("Tuning/Shooter/Readiness/Hood_Error", hoodError);
        Logger.recordOutput("Tuning/Shooter/Readiness/IsReady", rpmReady && turretReady && hoodReady);

        return rpmReady && turretReady && hoodReady;
    }

    public void setTurretAngle(double angleDeg) {
        turretTargetDeg = MathUtil.clamp(angleDeg, turretMinAngle.get(), turretMaxAngle.get());
        double encoderTarget = turretTargetDeg + turretCenterOffset.get();
        io.setTurretPosition(encoderTarget);
    }

    public double getTurretAngle() {
        return inputs.turretPositionDeg - turretCenterOffset.get();
    }

    public boolean isTurretAtTarget() {
        return Math.abs(getTurretAngle() - turretTargetDeg) < turretTolerance.get();
    }

    public void setHoodAngle(double angleDeg) {
        hoodTargetDeg = MathUtil.clamp(angleDeg, ShooterConstants.kHoodMinAngle, ShooterConstants.kHoodMaxAngle);
        io.setHoodPosition(hoodTargetDeg);
    }

    public double getHoodAngle() {
        return inputs.hoodPositionDeg;
    }

    public boolean isHoodAtTarget() {
        return Math.abs(getHoodAngle() - hoodTargetDeg) < hoodTolerance.get();
    }

    public void setFlywheelRPM(double rpm) {
        flywheelTargetRPM = Math.max(0, rpm);
        io.setFlywheelVelocityRPM(flywheelTargetRPM);
    }

    public double getFlywheelRPM() {
        return inputs.flywheelVelocityRPM;
    }

    public boolean isFlywheelAtTarget() {
        return Math.abs(getFlywheelRPM() - flywheelTargetRPM) < flywheelTolerance.get();
    }

    public void stopFlywheel() {
        flywheelTargetRPM = 0;
        io.stopFlywheel();
    }

    public void toggleAutoAim() {
        if (isAutoAimActive)
            disableAutoAim();
        else
            enableAutoAim();
    }

    public void enableAutoAim() {
        isAutoAimActive = true;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Tuning/Shooter/EnableAutoAim", true);
        tuningTable.getEntry("Tuning/Shooter/EnableAutoAim").setBoolean(true);
    }

    public void disableAutoAim() {
        isAutoAimActive = false;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Tuning/Shooter/EnableAutoAim", false);
        tuningTable.getEntry("Tuning/Shooter/EnableAutoAim").setBoolean(false);
        setTurretAngle(0);
    }

    public void adjustHoodOffset(double deltaDeg) {
        this.hoodOffsetDeg += deltaDeg;
    }

    public void adjustFlywheelOffset(double deltaRPM) {
        this.flywheelOffsetRPM += deltaRPM;
    }

    public double getHoodOffset() {
        return hoodOffsetDeg;
    }

    public double getFlywheelOffset() {
        return flywheelOffsetRPM;
    }

    private void trackTarget() {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));
        Translation2d targetLocation;
        boolean isPassing = !isInAllianceZone();

        if (isPassing)
            targetLocation = FieldConstants.getPassTarget(DriverStation.getAlliance(), robotPose);
        else
            targetLocation = getTargetHub();

        double distance = turretFieldPosition.getDistance(targetLocation);
        double angleToTargetRad = Math.atan2(targetLocation.getY() - turretFieldPosition.getY(),
                targetLocation.getX() - turretFieldPosition.getX());
        double targetTurretRad = angleToTargetRad - robotPose.getRotation().getRadians();

        if (edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Tuning/Shooter/InvertAiming", false)) {
            targetTurretRad += Math.PI;
        }
        targetTurretRad = MathUtil.angleModulus(targetTurretRad);
        turretTargetDeg = Math.toDegrees(targetTurretRad) + autoAimOffsetDeg;

        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg, turretMinAngle.get(), turretMaxAngle.get());
        }

        ShooterState state = isPassing ? calculateShooterStateForAlliancePass(distance)
                : calculateShooterState(distance);

        hoodTargetDeg = state.hoodAngleDeg + hoodOffsetDeg;
        flywheelTargetRPM = state.rpm + flywheelOffsetRPM;

        setTurretAngle(turretTargetDeg);
        setHoodAngle(hoodTargetDeg);
        setFlywheelRPM(flywheelTargetRPM);

        Logger.recordOutput("Tuning/Shooter/Aiming/Distance", distance);
        Logger.recordOutput("Tuning/Shooter/Aiming/TargetTurretAngle", turretTargetDeg);
    }

    public boolean isAutoAimActive() {
        return isAutoAimActive;
    }

    public Translation2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(alliance);
        double ovX = tunableTargetOverrideX.get();
        double ovY = tunableTargetOverrideY.get();
        if (Math.abs(ovX) > 0.01 || Math.abs(ovY) > 0.01)
            hubLocation = new Translation2d(ovX, ovY);
        return hubLocation;
    }

    public Translation2d getTurretCenterOfRotation() {
        return new Translation2d(turretPositionX.get(), turretPositionY.get());
    }

    public double getDistanceToHub() {
        return robotPoseSupplier.get().getTranslation().getDistance(getTargetHub());
    }

    public double getDistanceToPoint(Translation2d point) {
        return robotPoseSupplier.get().getTranslation().getDistance(point);
    }

    public boolean isInAllianceZone() {
        Pose2d robotPose = robotPoseSupplier.get();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red)
            return robotPose.getX() >= FieldConstants.kRedAllianceZoneMinX;
        else
            return robotPose.getX() <= FieldConstants.kBlueAllianceZoneMaxX;
    }

    public Translation2d getFeedingTarget() {
        return FieldConstants.getFeedingTarget(DriverStation.getAlliance());
    }

    public void setAutoAimOffset(double offsetDeg) {
        this.autoAimOffsetDeg = offsetDeg;
    }

    public void adjustAutoAimOffset(double deltaDeg) {
        this.autoAimOffsetDeg += deltaDeg;
    }

    public double getAutoAimOffset() {
        return autoAimOffsetDeg;
    }

    public void adjustHubOffset(double deltaX, double deltaY) {
        this.hubOffsetX += deltaX;
        this.hubOffsetY += deltaY;
    }

    public void resetHubOffset() {
        this.hubOffsetX = 0.0;
        this.hubOffsetY = 0.0;
    }

    public double getHubOffsetX() {
        return hubOffsetX;
    }

    public double getHubOffsetY() {
        return hubOffsetY;
    }

    private void logTelemetry() {
        Logger.recordOutput("Tuning/Shooter/Turret/ActualDeg", getTurretAngle());
        Logger.recordOutput("Tuning/Shooter/Hood/ActualDeg", getHoodAngle());
        Logger.recordOutput("Tuning/Shooter/Flywheel/ActualRPM", getFlywheelRPM());
        Logger.recordOutput("Tuning/Shooter/AutoAimActive", isAutoAimActive);
    }

    public void setMode(ShooterMode mode) {
        this.currentMode = mode;
    }

    public ShooterMode getMode() {
        return currentMode;
    }

    public void setTurretAngleTest(double angleDeg) {
        setTurretAngle(angleDeg);
    }

    public void setHoodAngleTest(double angleDeg) {
        setHoodAngle(angleDeg);
    }

    public double getFlywheelActualRPM() {
        return getFlywheelRPM();
    }

    public double getTurretActualAngle() {
        return getTurretAngle();
    }

    public double getHoodActualAngle() {
        return getHoodAngle();
    }

    public void stopShooter() {
        stopFlywheel();
        currentMode = ShooterMode.IDLE;
    }

    public void shoot() {
        currentMode = ShooterMode.SCORING;
    }

    public boolean isAimingAtTarget() {
        return isTurretAtTarget() && isHoodAtTarget() && isFlywheelAtTarget();
    }

    public void logSensorTelemetry() {
        logTelemetry();
    }

    public void initAutoShootCommand(frc.robot.subsystems.feeder.FeederSubsystem feeder,
            frc.robot.subsystems.drive.DriveSubsystem drive, frc.robot.subsystems.intake.IntakeSubsystem intake,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
                "Tuning/Shooter/Test/AutoShoot",
                new frc.robot.commands.shooter.AutoShootCommand(this, feeder, drive, intake, poseSupplier)
                        .withName("AutoShoot"));

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
                "Tuning/Shooter/Test/Reset Turret Offset",
                edu.wpi.first.wpilibj2.command.Commands
                        .runOnce(() -> this.adjustAutoAimOffset(-this.getAutoAimOffset()), this)
                        .ignoringDisable(true).withName("Reset Turret Offset"));
    }

    public Command getHomeHoodCommand() {
        return Commands.run(() -> io.setHoodVoltage(-2.0), this)
                .withTimeout(1.0)
                .finallyDo(() -> {
                    io.setHoodVoltage(0);
                    io.resetHoodEncoder(0.0);
                    System.out.println("[Shooter] Hood Homed.");
                });
    }

    public Command getHomeTurretCommand() {
        return new Command() {
            private double startPos;
            private int state;

            @Override
            public void initialize() {
                startPos = getTurretAngle();
                state = 0;
                addRequirements(ShooterSubsystem.this);
            }

            @Override
            public void execute() {
                if (inputs.turretReverseLimitSwitchPressed)
                    return;
                double currentPos = getTurretAngle();
                if (state == 0) {
                    io.setTurretVoltage(-0.7);
                    if (currentPos <= startPos - 100.0)
                        state = 1;
                } else if (state == 1) {
                    io.setTurretVoltage(0.7);
                    if (currentPos >= startPos + 100.0)
                        state = 2;
                }
            }

            @Override
            public boolean isFinished() {
                return inputs.turretReverseLimitSwitchPressed || state == 2;
            }

            @Override
            public void end(boolean interrupted) {
                io.setTurretVoltage(0);
                if (inputs.turretReverseLimitSwitchPressed) {
                    io.resetTurretEncoder();
                    setTurretAngle(0.0);
                }
            }
        }.withName("HomeTurret");
    }

    public void stopAll() {
        io.stopTurret();
        io.stopHood();
        stopFlywheel();
    }
}