package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOHardware implements IntakeIO {
    private final TalonFX rollerMotor;
    private final SparkMax extensionMotor;
    private final SparkClosedLoopController extensionPID;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final SparkMaxConfig extensionConfig = new SparkMaxConfig();

    public IntakeIOHardware() {
        rollerMotor = new TalonFX(RobotMap.kIntakeMotorID);
        extensionMotor = new SparkMax(RobotMap.kIntakePivotMotorID, MotorType.kBrushless);
        extensionPID = extensionMotor.getClosedLoopController();

        /* Initial application taken care by config methods via Subsystem */
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerVelocityRPM = rollerMotor.getVelocity().getValueAsDouble() * 60.0;
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerCurrentAmps = new double[] { rollerMotor.getStatorCurrent().getValueAsDouble() };

        inputs.extensionPositionRevs = extensionMotor.getEncoder().getPosition();
        inputs.extensionVelocityRPM = extensionMotor.getEncoder().getVelocity();
        inputs.extensionAppliedVolts = extensionMotor.getAppliedOutput() * extensionMotor.getBusVoltage();
        inputs.extensionCurrentAmps = new double[] { extensionMotor.getOutputCurrent() };
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setRollerVelocity(double rpm) {
        rollerMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void setExtensionPositionRevs(double revs) {
        extensionPID.setReference(revs, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    @Override
    public void setExtensionVoltage(double volts) {
        extensionMotor.setVoltage(volts);
    }

    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    @Override
    public void stopExtension() {
        extensionMotor.stopMotor();
    }

    @Override
    public void resetExtensionEncoder(double revs) {
        extensionMotor.getEncoder().setPosition(revs);
    }

    @Override
    public void configRoller(double kP, double kI, double kD, double kV, boolean invert) {
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.Slot0.kV = kV;
        rollerConfig.Slot0.kP = kP;
        rollerConfig.Slot0.kI = kI;
        rollerConfig.Slot0.kD = kD;

        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    @Override
    public void configExtension(double kP, double kI, double kD, double minRev, double maxRev, boolean softLimitsEnabled, boolean invert) {
        extensionConfig.inverted(invert);
        extensionConfig.idleMode(IdleMode.kCoast);
        extensionConfig.smartCurrentLimit(IntakeConstants.kExtensionCurrentLimit);

        extensionConfig.closedLoop.pid(kP, kI, kD);
        extensionConfig.encoder.positionConversionFactor(1.0);
        extensionConfig.encoder.velocityConversionFactor(1.0 / 60.0);

        extensionConfig.softLimit.reverseSoftLimit((float) minRev);
        extensionConfig.softLimit.reverseSoftLimitEnabled(softLimitsEnabled);
        extensionConfig.softLimit.forwardSoftLimit((float) maxRev);
        extensionConfig.softLimit.forwardSoftLimitEnabled(softLimitsEnabled);

        extensionConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                .primaryEncoderVelocityPeriodMs(500);

        extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
