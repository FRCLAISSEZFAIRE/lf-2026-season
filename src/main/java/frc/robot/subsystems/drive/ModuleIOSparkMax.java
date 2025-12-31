package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.ModuleConstants;

/**
 * MaxSwerve modül için Spark motor kontrol implementasyonu.
 * Drive: SparkFlex (Vortex)
 * Turn: SparkMax (Neo 550) + Absolute Encoder
 */
public class ModuleIOSparkMax implements ModuleIO {

    // Motorlar
    private final SparkFlex driveMotor;
    private final SparkMax turnMotor;

    // Encoderlar
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnAbsoluteEncoder;

    // Kontrolcü
    private final SparkClosedLoopController turnClosedLoopController;

    private final Rotation2d angleOffset;

    public ModuleIOSparkMax(int driveID, int turnID, Rotation2d angleOffset) {
        this.angleOffset = angleOffset;

        // -----------------------------------------------------------
        // 1. DRIVE MOTOR (Vortex - SparkFlex)
        // -----------------------------------------------------------
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        SparkFlexConfig driveConfig = new SparkFlexConfig();

        driveConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        driveConfig.idleMode(IdleMode.kBrake);

        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();

        // -----------------------------------------------------------
        // 2. TURN MOTOR (Neo 550 - SparkMax)
        // -----------------------------------------------------------
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        turnConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.inverted(ModuleConstants.kTurningEncoderInverted);

        // Absolute Encoder Ayarları
        turnConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turnConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        turnConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted);

        // PID Ayarları
        turnConfig.closedLoop.pid(
                ModuleConstants.kTurningP,
                ModuleConstants.kTurningI,
                ModuleConstants.kTurningD);

        // PID Wrapping (0 - 2*PI arasında)
        turnConfig.closedLoop.positionWrappingEnabled(true);
        turnConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor);

        turnMotor.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
        turnClosedLoopController = turnMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Drive
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        // Turn
        double absolutePosition = turnAbsoluteEncoder.getPosition() - angleOffset.getRadians();
        absolutePosition %= (2 * Math.PI);
        if (absolutePosition < 0)
            absolutePosition += (2 * Math.PI);

        inputs.turnAbsolutePositionRad = absolutePosition;
        inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnPosition(double angleRad) {
        double angleWithOffset = angleRad + angleOffset.getRadians();
        turnClosedLoopController.setReference(
                angleWithOffset,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
    }
}