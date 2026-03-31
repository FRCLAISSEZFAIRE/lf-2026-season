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

import frc.robot.util.TunableNumber;
import frc.robot.util.TunableBoolean;

public class DriveSubsystem extends SubsystemBase {
    // IO Layers
    private final GyroIO m_gyroIO;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_rearLeft;
    private final MAXSwerveModule m_rearRight;

    // Simulation State
    private Rotation2d m_simRotation = new Rotation2d();

    // Pose Estimator for tracking robot pose (supports Vision Fusion)
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Field visualization
    private final Field2d field = new Field2d();

    // Speed Multipliers
    private final TunableNumber teleopSpeedMultiplier = new TunableNumber("Tuning/Drive", "TeleopSpeedMultiplier", 1.0);
    private final TunableNumber autoSpeedMultiplier = new TunableNumber("Tuning/Auto", "SpeedMultiplier", 1.0);
    private final TunableNumber feedSpeedMultiplier = new TunableNumber("Tuning/Auto", "FeedSpeedMultiplier", 0.5);
    private boolean feedMode = false;

    // Module PID Tunables
    private final TunableNumber moduleDriveP = new TunableNumber("Tuning/Drive", "ModuleDrive_kP", 0.04);
    private final TunableNumber moduleTurnP = new TunableNumber("Tuning/Drive/Module", "Turn kP", 1.0);

    private final TunableNumber maxAccel = new TunableNumber("Tuning/Drive/Limits", "Max Accel (mps2)", 3.0);
    private final TunableNumber maxAngularAccelRad = new TunableNumber("Tuning/Drive/Limits", "Max Angular Accel (radps2)", 12.0);

    // LIVE INVERSION TUNABLES (Saved to RIO)
    private final TunableBoolean invertDriveFL = new TunableBoolean("Tuning/Drive/Inverts/Driving", "FL (1)", DriveConstants.kFrontLeftDrivingInverted);
    private final TunableBoolean invertDriveFR = new TunableBoolean("Tuning/Drive/Inverts/Driving", "FR (3)", DriveConstants.kFrontRightDrivingInverted);
    private final TunableBoolean invertDriveRL = new TunableBoolean("Tuning/Drive/Inverts/Driving", "RL (5)", DriveConstants.kRearLeftDrivingInverted);
    private final TunableBoolean invertDriveRR = new TunableBoolean("Tuning/Drive/Inverts/Driving", "RR (7)", DriveConstants.kRearRightDrivingInverted);

    private final TunableBoolean invertTurnFL = new TunableBoolean("Tuning/Drive/Inverts/Turning", "FL (2)", false);
    private final TunableBoolean invertTurnFR = new TunableBoolean("Tuning/Drive/Inverts/Turning", "FR (4)", false);
    private final TunableBoolean invertTurnRL = new TunableBoolean("Tuning/Drive/Inverts/Turning", "RL (6)", false);
    private final TunableBoolean invertTurnRR = new TunableBoolean("Tuning/Drive/Inverts/Turning", "RR (8)", false);

    // Safety Layer Configuration
    private static final double ROBOT_RADIUS = 0.6;
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;
    private boolean safeDriveEnabled = true;

    // Center of Rotation
    private Translation2d centerOfRotation = new Translation2d(0, 0);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO rlIO, ModuleIO rrIO) {
        this.m_gyroIO = gyroIO;
        
        m_frontLeft = new MAXSwerveModule(flIO, "FrontLeft", DriveConstants.kFrontLeftChassisAngularOffset);
        m_frontRight = new MAXSwerveModule(frIO, "FrontRight", DriveConstants.kFrontRightChassisAngularOffset);
        m_rearLeft = new MAXSwerveModule(rlIO, "RearLeft", DriveConstants.kRearLeftChassisAngularOffset);
        m_rearRight = new MAXSwerveModule(rrIO, "RearRight", DriveConstants.kRearRightChassisAngularOffset);

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
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9)
        );

        SmartDashboard.putData("Field", field);
        m_gyroIO.reset();
        SmartDashboard.putData("Drive/ResetGyro", new edu.wpi.first.wpilibj2.command.InstantCommand(this::zeroHeading));
        SmartDashboard.putBoolean("Drive/InvertGyro", DriveConstants.kGyroReversed);

        SmartDashboard.putData("Drive/PoseX+", new edu.wpi.first.wpilibj2.command.InstantCommand(() -> resetOdometry(new Pose2d(getPose().getX() + 0.10, getPose().getY(), getPose().getRotation())), this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseX-", new edu.wpi.first.wpilibj2.command.InstantCommand(() -> resetOdometry(new Pose2d(getPose().getX() - 0.10, getPose().getY(), getPose().getRotation())), this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseY+", new edu.wpi.first.wpilibj2.command.InstantCommand(() -> resetOdometry(new Pose2d(getPose().getX(), getPose().getY() + 0.10, getPose().getRotation())), this).ignoringDisable(true));
        SmartDashboard.putData("Drive/PoseY-", new edu.wpi.first.wpilibj2.command.InstantCommand(() -> resetOdometry(new Pose2d(getPose().getX(), getPose().getY() - 0.10, getPose().getRotation())), this).ignoringDisable(true));

        updateModuleInversions();
        updateModulePID();
    }

    private void updateModuleInversions() {
        m_frontLeft.updateInversions(invertDriveFL.get(), invertTurnFL.get());
        m_frontRight.updateInversions(invertDriveFR.get(), invertTurnFR.get());
        m_rearLeft.updateInversions(invertDriveRL.get(), invertTurnRL.get());
        m_rearRight.updateInversions(invertDriveRR.get(), invertTurnRR.get());
    }

    private void updateModulePID() {
        double dP = moduleDriveP.get();
        double tP = moduleTurnP.get();
        m_frontLeft.updatePID(dP, tP);
        m_frontRight.updatePID(dP, tP);
        m_rearLeft.updatePID(dP, tP);
        m_rearRight.updatePID(dP, tP);
    }

    public void zeroHeading() {
        m_gyroIO.reset();
        var alliance = DriverStation.getAlliance();
        Rotation2d targetHeading = new Rotation2d();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetHeading = Rotation2d.fromDegrees(180);
        }
        m_simRotation = targetHeading;
        m_gyroIO.setYaw(targetHeading.getDegrees());
        Pose2d currentPose = getPose();
        resetOdometry(new Pose2d(currentPose.getTranslation(), targetHeading));
    }

    @Override
    public void periodic() {
        // Update IOs
        m_gyroIO.updateInputs(m_gyroInputs);
        Logger.processInputs("Drive/Gyro", m_gyroInputs);
        
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();

        m_poseEstimator.update(
                getRawGyroRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        field.setRobotPose(getPose());

        Logger.recordOutput("Tuning/Drive/Pose", getPose());
        Logger.recordOutput("Tuning/Drive/Heading", getRotation2d());
        Logger.recordOutput("Tuning/Drive/HeadingDeg", getRotation2d().getDegrees());
        Logger.recordOutput("Tuning/Drive/RawGyroDeg", getRawGyroRotation2d().getDegrees());
        Logger.recordOutput("Tuning/Drive/GyroRate", m_gyroInputs.yawVelocityRadPerSec);

        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontLeft", m_frontLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/FrontRight", m_frontRight.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearLeft", m_rearLeft.getState());
        Logger.recordOutput("Tuning/Drive/ModuleStates/RearRight", m_rearRight.getState());

        if (moduleDriveP.hasChanged() || moduleTurnP.hasChanged()) updateModulePID();

        if (invertDriveFL.hasChanged() || invertTurnFL.hasChanged()) m_frontLeft.updateInversions(invertDriveFL.get(), invertTurnFL.get());
        if (invertDriveFR.hasChanged() || invertTurnFR.hasChanged()) m_frontRight.updateInversions(invertDriveFR.get(), invertTurnFR.get());
        if (invertDriveRL.hasChanged() || invertTurnRL.hasChanged()) m_rearLeft.updateInversions(invertDriveRL.get(), invertTurnRL.get());
        if (invertDriveRR.hasChanged() || invertTurnRR.hasChanged()) m_rearRight.updateInversions(invertDriveRR.get(), invertTurnRR.get());
    }

    public void showTargetPose(Pose2d target) { field.getObject("Target").setPose(target); }
    public Pose2d getPose() { return m_poseEstimator.getEstimatedPosition(); }

    public void resetOdometry(Pose2d pose) {
        m_simRotation = pose.getRotation();
        m_poseEstimator.resetPosition(
                getRawGyroRotation2d(),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition() },
                pose);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drive(new Translation2d(xSpeed, ySpeed), rot, fieldRelative);
    }

    public void drive(Translation2d translation, double rotationZ, boolean fieldRelative) {
        ChassisSpeeds targetSpeeds;
        if (fieldRelative) {
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationZ, getRotation2d());
        } else {
            targetSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotationZ);
        }
        if (safeDriveEnabled) targetSpeeds = applySafetyLayer(targetSpeeds, fieldRelative);
        driveRobotRelative(targetSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        double multiplier = 1.0;
        if (DriverStation.isAutonomous()) {
            multiplier = Math.max(0.1, Math.min(1.0, autoSpeedMultiplier.get()));
            if (feedMode) multiplier *= Math.max(0.1, Math.min(1.0, feedSpeedMultiplier.get()));
        } else if (DriverStation.isTeleop()) {
            multiplier = Math.max(0.1, Math.min(1.0, teleopSpeedMultiplier.get()));
        }
        speeds.vxMetersPerSecond *= multiplier;
        speeds.vyMetersPerSecond *= multiplier;
        speeds.omegaRadiansPerSecond *= multiplier;

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void runVelocity(ChassisSpeeds speeds) { driveRobotRelative(speeds); }
    public void setFeedMode(boolean enabled) { this.feedMode = enabled; }
    public boolean isFeedMode() { return feedMode; }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        if (rateLimit) driveManual(xSpeed, ySpeed, rot, fieldRelative);
        else drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private double m_prevTime = edu.wpi.first.util.WPIUtilJNI.now() * 1e-6;
    private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

    public void driveManual(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot);
        driveRobotRelative(limitRates(desiredSpeeds));
    }

    public ChassisSpeeds limitRates(ChassisSpeeds commandedSpeeds) {
        double currentMaxAccel = maxAccel.get();
        double currentMaxAngularAccelRad = maxAngularAccelRad.get();
        double currentTime = edu.wpi.first.util.WPIUtilJNI.now() * 1e-6;
        double dt = currentTime - m_prevTime;

        double accelerationDif = currentMaxAccel * dt;
        double xSpeed = MathUtil.clamp(commandedSpeeds.vxMetersPerSecond, m_lastSpeeds.vxMetersPerSecond - accelerationDif, m_lastSpeeds.vxMetersPerSecond + accelerationDif);
        double ySpeed = MathUtil.clamp(commandedSpeeds.vyMetersPerSecond, m_lastSpeeds.vyMetersPerSecond - accelerationDif, m_lastSpeeds.vyMetersPerSecond + accelerationDif);
        double thetaAccelDif = currentMaxAngularAccelRad * dt;
        double thetaSpeed = MathUtil.clamp(commandedSpeeds.omegaRadiansPerSecond, m_lastSpeeds.omegaRadiansPerSecond - thetaAccelDif, m_lastSpeeds.omegaRadiansPerSecond + thetaAccelDif);

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        m_lastSpeeds = limitedSpeeds;
        m_prevTime = currentTime;
        return limitedSpeeds;
    }

    public void stop() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setCenterOfRotation(Translation2d center) { this.centerOfRotation = center; }
    public void resetCenterOfRotation() { this.centerOfRotation = new Translation2d(0, 0); }
    public Translation2d getCenterOfRotation() { return centerOfRotation; }

    public ChassisSpeeds getRobotVelocity() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRotation2d());
    }

    public Rotation2d getRotation2d() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getRawGyroRotation2d() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation() && !m_gyroInputs.connected) {
            return m_simRotation;
        }
        boolean invertGyro = DriveConstants.kGyroReversed;
        if (invertGyro) {
            return Rotation2d.fromRadians(-m_gyroInputs.yawPositionRad);
        }
        return Rotation2d.fromRadians(m_gyroInputs.yawPositionRad);
    }

    public double getGyroVelocityRadPerSec() {
        return m_gyroInputs.yawVelocityRadPerSec;
    }

    private ChassisSpeeds applySafetyLayer(ChassisSpeeds speeds, boolean fieldRelative) {
        Pose2d currentPose = getPose();
        ChassisSpeeds fieldSpeeds = fieldRelative ? speeds : ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d());

        double fieldVx = fieldSpeeds.vxMetersPerSecond;
        double fieldVy = fieldSpeeds.vyMetersPerSecond;

        if (currentPose.getX() < ROBOT_RADIUS && fieldVx < 0) fieldVx = 0;
        if (currentPose.getX() > (FIELD_LENGTH - ROBOT_RADIUS) && fieldVx > 0) fieldVx = 0;
        if (currentPose.getY() < ROBOT_RADIUS && fieldVy < 0) fieldVy = 0;
        if (currentPose.getY() > (FIELD_WIDTH - ROBOT_RADIUS) && fieldVy > 0) fieldVy = 0;

        for (Translation2d zoneCenter : FieldConstants.kKeepOutZones) {
            double dist = currentPose.getTranslation().getDistance(zoneCenter);
            double safeZoneRadius = 0.5 + ROBOT_RADIUS;

            if (dist < safeZoneRadius) {
                Translation2d vecToZone = zoneCenter.minus(currentPose.getTranslation());
                double dotProduct = (fieldVx * vecToZone.getX()) + (fieldVy * vecToZone.getY());
                if (dotProduct > 0) { fieldVx = 0; fieldVy = 0; }
            }
        }
        ChassisSpeeds safeSpeeds = new ChassisSpeeds(fieldVx, fieldVy, speeds.omegaRadiansPerSecond);
        return fieldRelative ? safeSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(safeSpeeds, getRotation2d());
    }

    @Override
    public void simulationPeriodic() {
        double dt = 0.02;
        ChassisSpeeds speeds = getRobotVelocity();
        m_simRotation = m_simRotation.plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

        m_frontLeft.simulationPeriodic(dt);
        m_frontRight.simulationPeriodic(dt);
        m_rearLeft.simulationPeriodic(dt);
        m_rearRight.simulationPeriodic(dt);
    }
}