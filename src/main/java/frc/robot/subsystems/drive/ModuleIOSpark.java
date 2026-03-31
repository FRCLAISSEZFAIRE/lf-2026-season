package frc.robot.subsystems.drive;

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

public class ModuleIOSpark implements ModuleIO {
    private final SparkFlex m_drivingSpark; // Vortex
    private final SparkMax m_turningSpark; // Neo 550

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    public ModuleIOSpark(int drivingCANId, int turningCANId, boolean drivingInverted) {
        m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Base config from tuning
        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        if (drivingInverted) {
            SparkFlexConfig invertConfig = new SparkFlexConfig();
            invertConfig.inverted(true);
            m_drivingSpark.configure(invertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

        m_drivingEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = m_drivingEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = m_drivingEncoder.getVelocity();
        inputs.driveAppliedVolts = m_drivingSpark.getAppliedOutput() * m_drivingSpark.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { m_drivingSpark.getOutputCurrent() };

        inputs.turnAbsolutePositionRad = m_turningEncoder.getPosition();
        inputs.turnVelocityRadPerSec = m_turningEncoder.getVelocity();
        inputs.turnAppliedVolts = m_turningSpark.getAppliedOutput() * m_turningSpark.getBusVoltage();
        inputs.turnCurrentAmps = new double[] { m_turningSpark.getOutputCurrent() };
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        m_drivingClosedLoopController.setSetpoint(velocityMetersPerSec, ControlType.kVelocity);
    }

    @Override
    public void setTurnPosition(double positionRad) {
        m_turningClosedLoopController.setSetpoint(positionRad, ControlType.kPosition);
    }

    @Override
    public void updatePID(double driveP, double turnP) {
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.closedLoop.pid(driveP, 0.0, 0.0);
        m_drivingSpark.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.closedLoop.pid(turnP, 0.0, 0.0);
        m_turningSpark.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInversions(boolean driveInverted, boolean turnInverted) {
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.inverted(driveInverted);
        m_drivingSpark.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.inverted(turnInverted);
        m_turningSpark.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
