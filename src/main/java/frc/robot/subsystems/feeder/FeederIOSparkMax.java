package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.FeederConstants;

public class FeederIOSparkMax implements FeederIO {
    private final SparkMax indexerMotor;
    private final SparkMax kickerMotor;

    private final SparkMaxConfig indexerConfig = new SparkMaxConfig();
    private final SparkMaxConfig kickerConfig = new SparkMaxConfig();

    public FeederIOSparkMax() {
        indexerMotor = new SparkMax(FeederConstants.kIndexerMotorID, MotorType.kBrushless);
        kickerMotor = new SparkMax(FeederConstants.kKickerMotorID, MotorType.kBrushless);

        configureIndexer();
        configureKicker();
    }

    private void configureIndexer() {
        indexerConfig.encoder.velocityConversionFactor(1.0);
        indexerConfig.closedLoop
                .p(FeederConstants.kIndexerP)
                .i(FeederConstants.kIndexerI)
                .d(FeederConstants.kIndexerD)
                .velocityFF(FeederConstants.kIndexerFF)
                .outputRange(-1, 1);
        indexerConfig.inverted(false);
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerConfig.smartCurrentLimit(FeederConstants.kCurrentLimit);

        indexerConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                .primaryEncoderPositionPeriodMs(500);

        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureKicker() {
        kickerConfig.encoder.velocityConversionFactor(1.0);
        kickerConfig.closedLoop
                .p(FeederConstants.kKickerP)
                .i(FeederConstants.kKickerI)
                .d(FeederConstants.kKickerD)
                .velocityFF(FeederConstants.kKickerFF)
                .outputRange(-1, 1);
        kickerConfig.inverted(false);
        kickerConfig.idleMode(IdleMode.kCoast);
        kickerConfig.smartCurrentLimit(FeederConstants.kCurrentLimit);

        kickerConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                .primaryEncoderPositionPeriodMs(500);

        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.indexerVelocityRPM = indexerMotor.getEncoder().getVelocity();
        inputs.kickerVelocityRPM = kickerMotor.getEncoder().getVelocity();
        
        inputs.indexerAppliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        inputs.kickerAppliedVolts = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        
        inputs.indexerCurrentAmps = new double[] { indexerMotor.getOutputCurrent() };
        inputs.kickerCurrentAmps = new double[] { kickerMotor.getOutputCurrent() };
    }

    @Override
    public void setIndexerVelocity(double rpm) {
        indexerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setKickerVelocity(double rpm) {
        kickerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setIndexerVoltage(double volts) {
        indexerMotor.setVoltage(volts);
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }

    @Override
    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    @Override
    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    @Override
    public void configPID(double indexerP, double indexerI, double indexerD, double indexerFF,
                          double kickerP, double kickerI, double kickerD, double kickerFF) {
        indexerConfig.closedLoop.p(indexerP).i(indexerI).d(indexerD).velocityFF(indexerFF);
        indexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        kickerConfig.closedLoop.p(kickerP).i(kickerI).d(kickerD).velocityFF(kickerFF);
        kickerMotor.configure(kickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
