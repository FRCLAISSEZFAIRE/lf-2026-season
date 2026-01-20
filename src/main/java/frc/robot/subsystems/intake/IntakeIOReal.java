package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.VisionConstants;

/**
 * Intake mekanizması için gerçek donanım IO implementasyonu (REVLib 2026).
 * Roller: NEO (SparkMax)
 * Pivot: NEO (SparkMax) + Absolute Encoder (Position Control)
 * Vision: Limelight 3A (Object Detection)
 */
public class IntakeIOReal implements IntakeIO {
    private final SparkMax rollerMotor;
    private final SparkMax pivotMotor;
    private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private final RelativeEncoder pivotInternalEncoder;
    private final SparkClosedLoopController pivotController;

    private final String cameraName = VisionConstants.kLimelightName;

    public IntakeIOReal(int rollerMotorID) {
        // --- ROLLER MOTOR ---
        rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(40);
        rollerMotor.configure(rollerConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // --- PIVOT MOTOR ---
        pivotMotor = new SparkMax(MechanismConstants.kIntakePivotID, MotorType.kBrushless);
        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
        pivotInternalEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        // Default PID ile başlat
        configurePivotPID(1.0, 0.0, 0.0);

        // Seed pivot position from absolute encoder
        seedPivotPosition();
    }

    /**
     * Pivot PID değerlerini günceller.
     * TunableNumber değiştiğinde çağrılır.
     */
    public void configurePivotPID(double p, double i, double d) {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(40);

        // Encoder config
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI); // Radyan
        pivotConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

        // Closed loop config
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pivotConfig.closedLoop.p(p);
        pivotConfig.closedLoop.i(i);
        pivotConfig.closedLoop.d(d);
        pivotConfig.closedLoop.outputRange(-0.5, 0.5);

        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Pivot başlangıç pozisyonunu absolute encoder'dan seed eder.
     */
    public void seedPivotPosition() {
        double absoluteRotations = pivotAbsoluteEncoder.getPosition();
        double absoluteRadians = absoluteRotations * 2 * Math.PI;
        pivotInternalEncoder.setPosition(absoluteRadians);
        System.out.println("[Intake Pivot] Seeded position: " + Math.toDegrees(absoluteRadians) + "°");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Roller
        inputs.velocityRadPerSec = rollerMotor.getEncoder().getVelocity();
        inputs.currentAmps = rollerMotor.getOutputCurrent();
        inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();

        // Pivot - Internal encoder (seeded)
        inputs.pivotPositionRad = pivotInternalEncoder.getPosition();
        inputs.pivotVelocityRadPerSec = pivotInternalEncoder.getVelocity();
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

        inputs.intakeSensorTriggered = false;

        // Kamera
        double tv = LimelightHelpers.getTV(cameraName) ? 1.0 : 0.0;

        if (tv > 0.5) {
            inputs.hasGamePiece = true;
            inputs.targetTx = LimelightHelpers.getTX(cameraName);
            inputs.targetTy = LimelightHelpers.getTY(cameraName);
            inputs.targetArea = LimelightHelpers.getTA(cameraName);
            inputs.targetClass = LimelightHelpers.getNeuralClassID(cameraName);
        } else {
            inputs.hasGamePiece = false;
            inputs.targetTx = 0.0;
            inputs.targetTy = 0.0;
            inputs.targetArea = 0.0;
            inputs.targetClass = "";
        }
    }

    @Override
    public void setVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void setPivotPosition(double rad) {
        // REVLib 2026: setSetpoint
        pivotController.setSetpoint(rad, SparkBase.ControlType.kPosition);
    }
}