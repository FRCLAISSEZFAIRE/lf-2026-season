package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Configs;

public class MAXSwerveModule {
    private final SparkFlex m_drivingSpark; // Vortex
    private final SparkMax m_turningSpark; // Neo

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller.
     * 
     * @param drivingCANId         CAN ID for the driving motor (Vortex)
     * @param turningCANId         CAN ID for the turning motor (NEO 550)
     * @param chassisAngularOffset Angular offset for this module in radians
     * @param drivingInverted      true if driving motor should be inverted
     *                             (typically FR and RL)
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean drivingInverted) {
        m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Apply base configuration
        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Apply motor inversion if needed (FR and RL typically need inversion)
        if (drivingInverted) {
            SparkFlexConfig invertConfig = new SparkFlexConfig();
            invertConfig.inverted(true);
            m_drivingSpark.configure(invertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Legacy constructor for backwards compatibility (no inversion).
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        this(drivingCANId, turningCANId, chassisAngularOffset, false);
    }

    // Getters moved to bottom with simulation logic support

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        // Store the state actually sent to motors (optimized) for simulation
        m_desiredState = correctedDesiredState;
    }

    /**
     * Updates the PID gains for the driving and turning motors.
     * 
     * @param driveP Driving motor Proportional gain.
     * @param turnP  Turning motor Proportional gain.
     */
    public void updatePID(double driveP, double turnP) {
        // Create partial configs for updates
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.closedLoop.pid(driveP, 0.0, 0.0);
        m_drivingSpark.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.closedLoop.pid(turnP, 0.0, 0.0);
        m_turningSpark.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);

        // Sim reset
        m_simDrivePositionMeters = 0;
        m_simTurnPositionRad = 0;
    }

    // Simulation State
    private double m_simDrivePositionMeters = 0;
    private double m_simTurnPositionRad = 0;
    private double m_simDriveVelocityMetersPerSec = 0;

    public void simulationPeriodic(double dt) {
        // Simple simulation: Assume perfect tracking of desired state
        // Drive
        m_simDriveVelocityMetersPerSec = m_desiredState.speedMetersPerSecond;
        m_simDrivePositionMeters += m_simDriveVelocityMetersPerSec * dt;

        // Turn
        m_simTurnPositionRad = m_desiredState.angle.getRadians();
    }

    // Override getState and getPosition for simulation if needed,
    // OR just rely on setters updating the logic if we could mock encoders.
    // Since we can't easily mock REV encoders without vendor dep physics sim,
    // we will condition the getters or just update internal state that getters use?
    // REV's wrappers might return 0 in sim unless configured.
    // Let's modify getters to check for RobotBase.isSimulation() but that requires
    // import.
    // Better: Helper method to retrieve data.

    public SwerveModulePosition getPosition() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return new SwerveModulePosition(
                    m_simDrivePositionMeters,
                    new Rotation2d(m_simTurnPositionRad - m_chassisAngularOffset));
        }

        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public SwerveModuleState getState() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return new SwerveModuleState(
                    m_simDriveVelocityMetersPerSec,
                    new Rotation2d(m_simTurnPositionRad - m_chassisAngularOffset));
        }

        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }
}
