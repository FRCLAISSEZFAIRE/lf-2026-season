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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

// Studica NavX Import
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.util.TunableNumber;

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

    // Tunable Numbers
    private final TunableNumber autoTranslationP = new TunableNumber("Drive/Auto", "Translation kP", 1.0);
    private final TunableNumber autoRotationP = new TunableNumber("Drive/Auto", "Rotation kP", 1.0);

    private final TunableNumber moduleDriveP = new TunableNumber("Drive/Module", "Drive kP", 0.04);
    private final TunableNumber moduleTurnP = new TunableNumber("Drive/Module", "Turn kP", 1.0);

    private final TunableNumber maxAccel = new TunableNumber("Drive/Limits", "Max Accel (mps2)", 3.0);
    private final TunableNumber maxAngularAccelRad = new TunableNumber("Drive/Limits", "Max Angular Accel (radps2)",
            12.0);

    // Safety Layer Configuration
    private static final double ROBOT_RADIUS = 0.6; // 70x65cm frame + 8cm bumpers
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;
    private boolean safeDriveEnabled = true;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Initialize Pose Estimator
        // Start with standard deviations:
        // State (Odometry): 0.1m, 0.1m, 0.1rad (Trust internal sensors)
        // Vision: 0.9m, 0.9m, 0.9rad (Low trust initially)
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
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

        // Config PathPlanner
        setupPathPlanner();

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
     * PathPlanner AutoBuilder konfigürasyonu.
     */
    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(autoTranslationP.get(), 0.0, 0.0),
                            new PIDConstants(autoRotationP.get(), 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), false);
        }
    }

    @Override
    public void periodic() {
        // Update the pose estimator in the periodic block
        // Use getRotation2d() so it handles Simulation vs Real automatically
        m_poseEstimator.update(
                getRotation2d(),
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
        Logger.recordOutput("Tuning/Drive/Gyro", getRotation2d());

        // Module States
        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontLeft", m_frontLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontRight", m_frontRight.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearLeft", m_rearLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearRight", m_rearRight.getState());

        // === DIAGNOSTIC LOGGING for Odometry Inversion Debug ===
        Logger.recordOutput("Debug/Gyro/RawAngle", m_gyro.getAngle());
        Logger.recordOutput("Debug/Gyro/ComputedHeadingDeg", getRotation2d().getDegrees());
        Logger.recordOutput("Debug/Modules/FL_DrivePos", m_frontLeft.getPosition().distanceMeters);
        Logger.recordOutput("Debug/Modules/FR_DrivePos", m_frontRight.getPosition().distanceMeters);
        Logger.recordOutput("Debug/Modules/RL_DrivePos", m_rearLeft.getPosition().distanceMeters);
        Logger.recordOutput("Debug/Modules/RR_DrivePos", m_rearRight.getPosition().distanceMeters);

        // Dynamic Updates
        if (autoTranslationP.hasChanged() || autoRotationP.hasChanged()) {
            setupPathPlanner();
        }

        if (moduleDriveP.hasChanged() || moduleTurnP.hasChanged()) {
            double dP = moduleDriveP.get();
            double tP = moduleTurnP.get();
            m_frontLeft.updatePID(dP, tP);
            m_frontRight.updatePID(dP, tP);
            m_rearLeft.updatePID(dP, tP);
            m_rearRight.updatePID(dP, tP);
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
        m_gyro.reset();
        m_simRotation = pose.getRotation(); // Reset sim rotation too

        m_poseEstimator.resetPosition(
                getRotation2d(),
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
        // Convert ChassisSpeeds to ModuleStates
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
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

    public Rotation2d getRotation2d() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return m_simRotation;
        }

        // Use constant directly to avoid stale NetworkTables cache issues.
        // Dashboard toggle is still available for RUNTIME changes only.
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