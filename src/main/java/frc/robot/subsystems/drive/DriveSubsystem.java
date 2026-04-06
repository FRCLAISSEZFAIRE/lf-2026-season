package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;

import org.littletonrobotics.junction.Logger;

// Studica NavX Import
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.util.TunableNumber;
import frc.robot.util.TunableBoolean;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    // Motor inversion settings are in DriveConstants
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset,
            DriveConstants.kFrontLeftDrivingInverted);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset,
            DriveConstants.kFrontRightDrivingInverted);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kRearLeftChassisAngularOffset,
            DriveConstants.kRearLeftDrivingInverted);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kRearRightChassisAngularOffset,
            DriveConstants.kRearRightDrivingInverted);

    // The gyro sensor (NavX2 via Studica lib)
    private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

    // Simulation State
    private Rotation2d m_simRotation = new Rotation2d();

    // Pose Estimator for tracking robot pose (supports Vision Fusion)
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Field visualization
    private final Field2d field = new Field2d();

    // Speed Multipliers
    private final TunableNumber teleopSpeedMultiplier = new TunableNumber("Tuning/Drive", "TeleopSpeedMultiplier", 1.0);
    private final TunableNumber autoSpeedMultiplier = new TunableNumber("Tuning/Auto", "SpeedMultiplier", 1.0);

    // Feed Speed Multiplier — Feed/Intake toplama sırasında ek yavaşlatma
    private final TunableNumber feedSpeedMultiplier = new TunableNumber("Tuning/Auto", "FeedSpeedMultiplier", 0.5);
    private boolean feedMode = false;

    // Shoot Speed Multiplier — Atış sırasında ek yavaşlatma (atış hassasiyeti için)
    private final TunableNumber shootSpeedMultiplier = new TunableNumber("Tuning/Drive", "ShootSpeedMultiplier", 0.4);
    private boolean shootMode = false;

    // Module PID Tunables
    private final TunableNumber moduleDriveP = new TunableNumber("Tuning/Drive", "ModuleDrive_kP", 0.04);
    private final TunableNumber moduleTurnP = new TunableNumber("Drive/Module", "Turn kP", 1.0);

    private final TunableNumber maxAccel = new TunableNumber("Drive/Limits", "Max Accel (mps2)", 3.0);
    private final TunableNumber maxAngularAccelRad = new TunableNumber("Drive/Limits", "Max Angular Accel (radps2)",
            12.0);

    // ===========================================================================
    // LIVE INVERSION TUNABLES (Saved to RIO)
    // ===========================================================================
    // Driving Motors (SparkFlex)
    private final TunableBoolean invertDriveFL = new TunableBoolean("Drive/Inverts/Driving", "FL (1)",
            DriveConstants.kFrontLeftDrivingInverted);
    private final TunableBoolean invertDriveFR = new TunableBoolean("Drive/Inverts/Driving", "FR (3)",
            DriveConstants.kFrontRightDrivingInverted);
    private final TunableBoolean invertDriveRL = new TunableBoolean("Drive/Inverts/Driving", "RL (5)",
            DriveConstants.kRearLeftDrivingInverted);
    private final TunableBoolean invertDriveRR = new TunableBoolean("Drive/Inverts/Driving", "RR (7)",
            DriveConstants.kRearRightDrivingInverted);

    // Turning Motors (SparkMax)
    private final TunableBoolean invertTurnFL = new TunableBoolean("Drive/Inverts/Turning", "FL (2)", false);
    private final TunableBoolean invertTurnFR = new TunableBoolean("Drive/Inverts/Turning", "FR (4)", false);
    private final TunableBoolean invertTurnRL = new TunableBoolean("Drive/Inverts/Turning", "RL (6)", false);
    private final TunableBoolean invertTurnRR = new TunableBoolean("Drive/Inverts/Turning", "RR (8)", false);

    // Safety Layer Configuration
    private static final double ROBOT_RADIUS = 0.6; // 70x65cm frame + 8cm bumpers
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;
    private boolean safeDriveEnabled = true;

    // Center of Rotation (robot frame, varsayılan: robot merkezi)
    private Translation2d centerOfRotation = new Translation2d(0, 0);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Initialize Pose Estimator
        // Start with standard deviations:
        // State (Odometry): 0.1m, 0.1m, 0.1rad (Trust internal sensors)
        // Vision: 0.9m, 0.9m, 0.9rad (Low trust initially)
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRawGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1), // State StdDevs
                VecBuilder.fill(0.9, 0.9, 0.9) // Vision StdDevs
        );

        SmartDashboard.putData("Field", field);

        // Reset Gyro on startup
        m_gyro.reset();

        // Put Reset Gyro command to Dashboard
        SmartDashboard.putData("Drive/ResetGyro", new edu.wpi.first.wpilibj2.command.InstantCommand(this::zeroHeading));

        // Initialize Gyro Inversion setting from Constants (putBoolean to force-update
        // stale cache)
        SmartDashboard.putBoolean("Drive/InvertGyro", DriveConstants.kGyroReversed);

        // Pose Offset — 10cm adımlarla X/Y kaydırma butonları
        SmartDashboard.putData("Drive/PoseX+",
                new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
                    Pose2d c = getPose();
                    resetOdometry(new Pose2d(c.getX() + 0.10, c.getY(), c.getRotation()));
                }, this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseX-",
                new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
                    Pose2d c = getPose();
                    resetOdometry(new Pose2d(c.getX() - 0.10, c.getY(), c.getRotation()));
                }, this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseY+",
                new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
                    Pose2d c = getPose();
                    resetOdometry(new Pose2d(c.getX(), c.getY() + 0.10, c.getRotation()));
                }, this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseY-",
                new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
                    Pose2d c = getPose();
                    resetOdometry(new Pose2d(c.getX(), c.getY() - 0.10, c.getRotation()));
                }, this).ignoringDisable(true));

        // Apply saved TunableNumber values for inversions and PID
        m_frontLeft.updateInversions(invertDriveFL.get(), invertTurnFL.get());
        m_frontRight.updateInversions(invertDriveFR.get(), invertTurnFR.get());
        m_rearLeft.updateInversions(invertDriveRL.get(), invertTurnRL.get());
        m_rearRight.updateInversions(invertDriveRR.get(), invertTurnRR.get());

        double initDP = moduleDriveP.get();
        double initTP = moduleTurnP.get();
        m_frontLeft.updatePID(initDP, initTP);
        m_frontRight.updatePID(initDP, initTP);
        m_rearLeft.updatePID(initDP, initTP);
        m_rearRight.updatePID(initDP, initTP);
    }

    /**
     * Resets the gyro heading.
     * Blue Alliance: 0 degrees.
     * Red Alliance: 180 degrees.
     */
    public void zeroHeading() {
        m_gyro.reset();

        // Determine heading based on Alliance
        var alliance = DriverStation.getAlliance();
        Rotation2d targetHeading = new Rotation2d(); // Default 0 (Blue)

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetHeading = Rotation2d.fromDegrees(180);
        }

        // Update Simulation State
        m_simRotation = targetHeading;

        // Reset Pose Estimator heading while keeping position
        Pose2d currentPose = getPose();
        resetOdometry(new Pose2d(currentPose.getTranslation(), targetHeading));

        System.out.println("[DriveSubsystem] Gyro Reset to " + targetHeading.getDegrees() + " degrees ("
                + (alliance.isPresent() ? alliance.get() : "Unknown") + ")");
    }

    /**
     * Resets the robot's field-relative rotation to a specific angle.
     * Keeps the current translation (position) unchanged.
     */
    public void resetRotation(Rotation2d rotation) {
        Pose2d currentPose = getPose();
        resetOdometry(new Pose2d(currentPose.getTranslation(), rotation));
        System.out.println("[DriveSubsystem] Rotation manual reset to " + rotation.getDegrees() + " degrees.");
    }

    @Override
    public void periodic() {
        // Update the pose estimator in the periodic block
        // Raw gyro kullanılmalı — getRotation2d() PoseEstimator'dan okuyor (circular
        // olur)
        m_poseEstimator.update(
                getRawGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        // Update field visualization
        field.setRobotPose(getPose());

        // Logging
        Logger.recordOutput("Tuning/Drive/Pose", getPose());
        Logger.recordOutput("Tuning/Drive/Heading", getRotation2d());
        Logger.recordOutput("Tuning/Drive/HeadingDeg", getRotation2d().getDegrees());
        Logger.recordOutput("Tuning/Drive/RawGyroDeg", getRawGyroRotation2d().getDegrees());
        Logger.recordOutput("Tuning/Drive/GyroRate", m_gyro.getRate());

        // Module States
        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontLeft", m_frontLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontRight", m_frontRight.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearLeft", m_rearLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearRight", m_rearRight.getState());

        if (moduleDriveP.hasChanged() || moduleTurnP.hasChanged()) {
            double dP = moduleDriveP.get();
            double tP = moduleTurnP.get();
            m_frontLeft.updatePID(dP, tP);
            m_frontRight.updatePID(dP, tP);
            m_rearLeft.updatePID(dP, tP);
            m_rearRight.updatePID(dP, tP);
        }

        // Check for inversion changes
        if (invertDriveFL.hasChanged() || invertTurnFL.hasChanged()) {
            m_frontLeft.updateInversions(invertDriveFL.get(), invertTurnFL.get());
        }
        if (invertDriveFR.hasChanged() || invertTurnFR.hasChanged()) {
            m_frontRight.updateInversions(invertDriveFR.get(), invertTurnFR.get());
        }
        if (invertDriveRL.hasChanged() || invertTurnRL.hasChanged()) {
            m_rearLeft.updateInversions(invertDriveRL.get(), invertTurnRL.get());
        }
        if (invertDriveRR.hasChanged() || invertTurnRR.hasChanged()) {
            m_rearRight.updateInversions(invertDriveRR.get(), invertTurnRR.get());
        }
    }

    public void showTargetPose(Pose2d target) {
        field.getObject("Target").setPose(target);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // NOT: Gyro burada sıfırlanmıyor! PoseEstimator, mevcut gyro açısını
        // referans alarak yeni pose'a göre offset hesaplar.
        // Gyro sadece zeroHeading() ile acil durumda sıfırlanır.
        m_simRotation = pose.getRotation(); // Reset sim rotation too

        m_poseEstimator.resetPosition(
                getRawGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Updates the pose estimator with a vision measurement.
     * 
     * @param pose      The pose estimated by vision.
     * @param timestamp The timestamp of the vision measurement in seconds.
     * @param stdDevs   Standard deviations [x, y, theta] for the measurement.
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp,
            edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drive(new Translation2d(xSpeed, ySpeed), rot, fieldRelative);
    }

    public void drive(Translation2d translation, double rotationZ, boolean fieldRelative) {
        ChassisSpeeds targetSpeeds;
        if (fieldRelative) {
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotationZ,
                    getRotation2d()); // uses odometry rotation
        } else {
            targetSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotationZ);
        }

        // Safety Layer
        if (safeDriveEnabled) {
            targetSpeeds = applySafetyLayer(targetSpeeds, fieldRelative);
        }

        driveRobotRelative(targetSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Speed Multiplier (0.1 to 1.0)
        double multiplier = 1.0;
        if (DriverStation.isAutonomous()) {
            multiplier = Math.max(0.1, Math.min(1.0, autoSpeedMultiplier.get()));
            // Feed mode: ek yavaşlatma çarpanı uygula
            if (feedMode) {
                multiplier *= Math.max(0.1, Math.min(1.0, feedSpeedMultiplier.get()));
            }
        } else if (DriverStation.isTeleop()) {
            multiplier = Math.max(0.1, Math.min(1.0, teleopSpeedMultiplier.get()));
        }

        // Shoot mode: atış sırasında yavaşlatma (Teleop ve Auto fark etmeksizin)
        if (shootMode) {
            multiplier *= Math.max(0.1, Math.min(1.0, shootSpeedMultiplier.get()));
        }

        speeds.vxMetersPerSecond *= multiplier;
        speeds.vyMetersPerSecond *= multiplier;
        speeds.omegaRadiansPerSecond *= multiplier;

        // Convert ChassisSpeeds to ModuleStates with center of rotation
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics
                .toSwerveModuleStates(speeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Compatibility: runVelocity was an alias for driveRobotRelative
    public void runVelocity(ChassisSpeeds speeds) {
        driveRobotRelative(speeds);
    }

    /** Feed modu: intake toplama sırasında ek yavaşlatma çarpanı aktif eder */
    public void setFeedMode(boolean enabled) {
        this.feedMode = enabled;
    }

    public boolean isFeedMode() {
        return feedMode;
    }

    /** Shoot modu: atış sırasında ek yavaşlatma çarpanı aktif eder */
    public void setShootMode(boolean enabled) {
        this.shootMode = enabled;
    }

    public boolean isShootMode() {
        return shootMode;
    }

    // Compatibility: 5-arg drive for AutoClimbCommand (vX, vY, rot, fieldRelative,
    // rateLimit)
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        if (rateLimit) {
            driveManual(xSpeed, ySpeed, rot, fieldRelative);
        } else {
            drive(xSpeed, ySpeed, rot, fieldRelative);
        }
    }

    /**
     * Legacy Manual Drive with Rate Limiting
     */
    private double m_prevTime = edu.wpi.first.util.WPIUtilJNI.now() * 1e-6;
    private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

    public void driveManual(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // 1. Calculate Desired ChassisSpeeds
        ChassisSpeeds desiredSpeeds;
        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        } else {
            desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // 2. Limit Rates (Acceleration Control)
        ChassisSpeeds limitedSpeeds = limitRates(desiredSpeeds);

        // 3. Drive
        driveRobotRelative(limitedSpeeds);
    }

    public ChassisSpeeds limitRates(ChassisSpeeds commandedSpeeds) {
        double currentMaxAccel = maxAccel.get();
        double currentMaxAngularAccelRad = maxAngularAccelRad.get();

        double currentTime = edu.wpi.first.util.WPIUtilJNI.now() * 1e-6;
        double dt = currentTime - m_prevTime;

        double accelerationDif = currentMaxAccel * dt;
        double xSpeed = MathUtil.clamp(commandedSpeeds.vxMetersPerSecond,
                m_lastSpeeds.vxMetersPerSecond - accelerationDif,
                m_lastSpeeds.vxMetersPerSecond + accelerationDif);

        double ySpeed = MathUtil.clamp(commandedSpeeds.vyMetersPerSecond,
                m_lastSpeeds.vyMetersPerSecond - accelerationDif,
                m_lastSpeeds.vyMetersPerSecond + accelerationDif);

        double thetaAccelDif = currentMaxAngularAccelRad * dt;
        double thetaSpeed = MathUtil.clamp(commandedSpeeds.omegaRadiansPerSecond,
                m_lastSpeeds.omegaRadiansPerSecond - thetaAccelDif,
                m_lastSpeeds.omegaRadiansPerSecond + thetaAccelDif);

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        m_lastSpeeds = limitedSpeeds;
        m_prevTime = currentTime;

        return limitedSpeeds;
    }

    /**
     * Sets the wheels into an X configuration to prevent movement.
     */
    public void stop() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // =====================================================================
    // CENTER OF ROTATION
    // =====================================================================

    /**
     * Dönüş merkezini ayarlar (robot frame, metre).
     * Atış sırasında taret merkezine ayarlanır.
     *
     * @param center Robot merkezine göre dönüş noktası (x=ileri, y=sol)
     */
    public void setCenterOfRotation(Translation2d center) {
        this.centerOfRotation = center;
    }

    /**
     * Dönüş merkezini robot merkezine (0,0) sıfırlar.
     */
    public void resetCenterOfRotation() {
        this.centerOfRotation = new Translation2d(0, 0);
    }

    /**
     * Mevcut dönüş merkezini döndürür.
     */
    public Translation2d getCenterOfRotation() {
        return centerOfRotation;
    }

    public ChassisSpeeds getRobotVelocity() {
        // Compute robot velocity from module states
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public ChassisSpeeds getFieldVelocity() {
        // Robot velocity rotated by heading
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRotation2d());
    }

    /**
     * Robot heading'ini döndürür (PoseEstimator'dan).
     * Field-relative sürüş ve tüm heading-bağımlı işlemler bu metodu kullanır.
     * Bu değer alliance-aware'dir: Red=180°, Blue=0° başlangıç.
     */
    public Rotation2d getRotation2d() {
        // PoseEstimator heading'ini kullan — gyro offset'lerini otomatik hesaplar.
        // Bu sayede resetOdometry(180°) sonrası field-relative doğru çalışır.
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Ham gyro açısını döndürür (PoseEstimator'dan bağımsız).
     * Sadece PoseEstimator.update() ve resetPosition() için kullanılır.
     * Field-relative sürüş için getRotation2d() kullanın.
     */
    public Rotation2d getRawGyroRotation2d() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return m_simRotation;
        }
        boolean invertGyro = DriveConstants.kGyroReversed;
        if (invertGyro) {
            return Rotation2d.fromDegrees(-m_gyro.getAngle());
        }
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }

    // Compatibility for VisionSubsystem
    public double getGyroVelocityRadPerSec() {
        return Math.toRadians(m_gyro.getRate());
    }

    // ===========================================================================
    // SAFETY LAYER
    // ===========================================================================
    private ChassisSpeeds applySafetyLayer(ChassisSpeeds speeds, boolean fieldRelative) {
        Pose2d currentPose = getPose();
        ChassisSpeeds fieldSpeeds = fieldRelative ? speeds
                : ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d());

        double fieldVx = fieldSpeeds.vxMetersPerSecond;
        double fieldVy = fieldSpeeds.vyMetersPerSecond;

        // X Axis boundary
        if (currentPose.getX() < ROBOT_RADIUS && fieldVx < 0)
            fieldVx = 0;
        if (currentPose.getX() > (FIELD_LENGTH - ROBOT_RADIUS) && fieldVx > 0)
            fieldVx = 0;

        // Y Axis boundary
        if (currentPose.getY() < ROBOT_RADIUS && fieldVy < 0)
            fieldVy = 0;
        if (currentPose.getY() > (FIELD_WIDTH - ROBOT_RADIUS) && fieldVy > 0)
            fieldVy = 0;

        // Keep-Out Zones
        for (Translation2d zoneCenter : FieldConstants.kKeepOutZones) {
            double dist = currentPose.getTranslation().getDistance(zoneCenter);
            double safeZoneRadius = 0.5 + ROBOT_RADIUS;

            if (dist < safeZoneRadius) {
                Translation2d vecToZone = zoneCenter.minus(currentPose.getTranslation());
                double dotProduct = (fieldVx * vecToZone.getX()) + (fieldVy * vecToZone.getY());
                if (dotProduct > 0) {
                    fieldVx = 0;
                    fieldVy = 0;
                }
            }
        }
        ChassisSpeeds safeSpeeds = new ChassisSpeeds(fieldVx, fieldVy, speeds.omegaRadiansPerSecond);
        return fieldRelative ? safeSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(safeSpeeds, getRotation2d());
    }

    @Override
    public void simulationPeriodic() {
        // Kinematic Simulation
        double dt = 0.02;
        ChassisSpeeds speeds = getRobotVelocity(); // From module states

        // Update Sim Rotation (Integrate angular velocity)
        m_simRotation = m_simRotation.plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

        // Update Module Simulation
        m_frontLeft.simulationPeriodic(dt);
        m_frontRight.simulationPeriodic(dt);
        m_rearLeft.simulationPeriodic(dt);
        m_rearRight.simulationPeriodic(dt);

        // Sim Gyro Device update (optional, usually unrelated to logic if we use
        // m_simRotation)
        int dev = edu.wpi.first.hal.SimDeviceJNI.createSimDevice("navX-Sensor[0]");
    }
}