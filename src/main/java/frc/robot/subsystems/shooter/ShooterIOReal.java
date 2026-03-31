package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

    private final SparkMax turretMotor;
    private final SparkMax hoodMotor;
    private final TalonFX flywheelMotor;

    private final SparkMaxConfig turretConfig = new SparkMaxConfig();
    private final SparkMaxConfig hoodConfig = new SparkMaxConfig();
    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

    public ShooterIOReal() {
        turretMotor = new SparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
        hoodMotor = new SparkMax(RobotMap.kHoodMotorID, MotorType.kBrushless);
        flywheelMotor = new TalonFX(RobotMap.kShooterMasterID);

        // Intial configs
        configTurretGearRatio(ShooterConstants.kTurretGearRatio);
        configHoodGearRatio(ShooterConstants.kHoodGearRatio);
        initFlywheel();

        turretMotor.getEncoder().setPosition(0);
        hoodMotor.getEncoder().setPosition(ShooterConstants.kHoodHomeAngle);
    }

    private void initFlywheel() {
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.turretPositionDeg = turretMotor.getEncoder().getPosition();
        inputs.turretVelocityRPM = turretMotor.getEncoder().getVelocity();
        inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretCurrentAmps = new double[] { turretMotor.getOutputCurrent() };
        inputs.turretReverseLimitSwitchPressed = turretMotor.getReverseLimitSwitch().isPressed();

        inputs.hoodPositionDeg = hoodMotor.getEncoder().getPosition();
        inputs.hoodVelocityRPM = hoodMotor.getEncoder().getVelocity();
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodCurrentAmps = new double[] { hoodMotor.getOutputCurrent() };

        inputs.flywheelVelocityRPM = flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = new double[] { flywheelMotor.getStatorCurrent().getValueAsDouble() };
    }

    @Override
    public void setTurretPosition(double degrees) {
        turretMotor.getClosedLoopController().setSetpoint(degrees, ControlType.kPosition);
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    @Override
    public void resetTurretEncoder() {
        turretMotor.getEncoder().setPosition(0.0);
    }

    @Override
    public void setHoodPosition(double degrees) {
        hoodMotor.getClosedLoopController().setSetpoint(degrees, ControlType.kPosition);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    @Override
    public void resetHoodEncoder(double degrees) {
        hoodMotor.getEncoder().setPosition(degrees);
    }

    @Override
    public void setFlywheelVelocityRPM(double rpm) {
        double rps = rpm / 60.0;
        flywheelMotor.setControl(flywheelVelocity.withVelocity(rps));
    }

    @Override
    public void stopTurret() {
        turretMotor.stopMotor();
    }

    @Override
    public void stopHood() {
        hoodMotor.stopMotor();
    }

    @Override
    public void stopFlywheel() {
        flywheelMotor.stopMotor();
    }

    @Override
    public void setTurretPID(double p, double i, double d, double maxOutput) {
        turretConfig.closedLoop.p(p).i(i).d(d).outputRange(-maxOutput, maxOutput);
        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTurretSoftLimits(double minAngle, double maxAngle) {
        turretConfig.softLimit.forwardSoftLimit((float) maxAngle).reverseSoftLimit((float) minAngle);
        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setHoodPID(double p, double i, double d) {
        hoodConfig.closedLoop.p(p).i(i).d(d);
        hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFlywheelPID(double p, double i, double d, double v) {
        flywheelConfig.Slot0.kP = p;
        flywheelConfig.Slot0.kI = i;
        flywheelConfig.Slot0.kD = d;
        flywheelConfig.Slot0.kV = v;
        flywheelMotor.getConfigurator().apply(flywheelConfig);
    }

    @Override
    public void configTurretGearRatio(double gearRatio) {
        double degreesPerMotorRot = 360.0 / gearRatio;
        turretConfig.encoder.positionConversionFactor(degreesPerMotorRot)
                            .velocityConversionFactor(degreesPerMotorRot / 60.0);

        turretConfig.inverted(true);
        turretConfig.idleMode(IdleMode.kBrake);

        turretConfig.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
        turretConfig.limitSwitch.reverseLimitSwitchEnabled(false)
                .reverseLimitSwitchType(com.revrobotics.spark.config.LimitSwitchConfig.Type.kNormallyOpen);

        turretConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                .primaryEncoderVelocityPeriodMs(500);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void configHoodGearRatio(double gearRatio) {
        double degreesPerMotorRot = 360.0 / gearRatio;
        hoodConfig.encoder.positionConversionFactor(degreesPerMotorRot)
                          .velocityConversionFactor(degreesPerMotorRot / 60.0);

        hoodConfig.inverted(true);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.smartCurrentLimit(ShooterConstants.kHoodCurrentLimit);

        hoodConfig.softLimit.forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float) ShooterConstants.kHoodMaxAngle)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float) ShooterConstants.kHoodMinAngle);

        hoodConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                .primaryEncoderVelocityPeriodMs(500);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFlywheelInverted(boolean inverted) {
        InvertedValue newInvertValue = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        flywheelConfig.MotorOutput.Inverted = newInvertValue;
        flywheelMotor.getConfigurator().apply(flywheelConfig);
    }
}
