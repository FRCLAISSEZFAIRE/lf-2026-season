package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.robot.LimelightHelpers;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.VisionConstants;

/**
 * Intake mekanizması için gerçek donanım IO implementasyonu.
 * Roller: NEO (SparkMax)
 * Pivot: NEO (SparkMax) + Absolute Encoder (Position Control)
 * Vision: Limelight 3A (Object Detection)
 */
public class IntakeIOReal implements IntakeIO {
    private final SparkMax rollerMotor;
    private final SparkMax pivotMotor;
    private final SparkAbsoluteEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    private final String cameraName = VisionConstants.kLimelightName;

    public IntakeIOReal(int rollerMotorID) {
        // --- ROLLER MOTOR ---
        rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(40);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- PIVOT MOTOR ---
        pivotMotor = new SparkMax(MechanismConstants.kIntakePivotID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(40);

        // Absolute Encoder Config (0-2PI radyan için scale factor gerekebilir ama
        // varsayılan 0-1 olabilir)
        // Burada basit PID ayarları yapıyoruz. Tuning gerekebilir.
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotConfig.closedLoop.pid(1.0, 0.0, 0.0); // P=1.0 placeholder
        pivotConfig.closedLoop.outputRange(-0.5, 0.5); // Güvenlik için hız limiti

        // Encoder Conversion (Eğer encoder 0-1 veriyorsa Radyana çevirelim)
        // SparkMax Absolute Encoder varsayılan 0-1 range verir mi? Evet.
        pivotConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        pivotConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60.0);
        pivotConfig.absoluteEncoder.inverted(false);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // --- Roller Verileri ---
        inputs.velocityRadPerSec = rollerMotor.getEncoder().getVelocity();
        inputs.currentAmps = rollerMotor.getOutputCurrent();
        inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();

        // --- Pivot Verileri ---
        inputs.pivotPositionRad = pivotEncoder.getPosition();
        inputs.pivotVelocityRadPerSec = pivotEncoder.getVelocity();
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

        // --- MZ80 Sensör (İptal) ---
        inputs.intakeSensorTriggered = false;

        // --- Kamera Verileri (Object Detection) ---
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
        pivotController.setReference(rad, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }
}