package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.RobotMap;

/**
 * Climber Subsystem (Modernized)
 * Uses Dual NEO (SparkMax) with Position Control
 */
public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;

    private final DigitalInput seatSensor;

    public enum ClimberPreset {
        HOME(ClimberConstants.kHomePosition),
        EXTEND(ClimberConstants.kClimbExtendPosition),
        RETRACT(ClimberConstants.kClimbRetractPosition),
        HOLD(ClimberConstants.kClimbHoldPosition);

        final double rot;

        ClimberPreset(double rot) {
            this.rot = rot;
        }
    }

    public ClimberSubsystem() {
        seatSensor = new DigitalInput(ClimberConstants.kSeatSensorDIO);

        leftMotor = new SparkMax(RobotMap.kClimberLeftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.kClimberRightMotorID, MotorType.kBrushless);

        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        configureMotor(leftMotor, false);
        configureMotor(rightMotor, true);
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ClimberConstants.kSupplyCurrentLimit);

        // Invert
        config.inverted(inverted);

        // PID
        config.closedLoop.pid(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD);
        config.closedLoop.outputRange(-1, 1);

        // Soft Limits
        config.softLimit.forwardSoftLimit(ClimberConstants.kForwardSoftLimit);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(ClimberConstants.kReverseSoftLimit);
        config.softLimit.reverseSoftLimitEnabled(true);

        // Position Conversion (Default 1.0 is OK for Rotations)
        config.encoder.positionConversionFactor(1.0);
        config.encoder.velocityConversionFactor(1.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0); // Assume Home
    }

    @Override
    public void periodic() {
        logTelemetry();
    }

    private void logTelemetry() {
        org.littletonrobotics.junction.Logger.recordOutput("Climber/LeftPosition",
                leftMotor.getEncoder().getPosition());
        org.littletonrobotics.junction.Logger.recordOutput("Climber/RightPosition",
                rightMotor.getEncoder().getPosition());
        org.littletonrobotics.junction.Logger.recordOutput("Climber/IsSeated", seatSensor.get());
    }

    // =========================================================================
    // COMMANDS
    // =========================================================================

    public void setPosition(double rotations) {
        leftPID.setReference(rotations, ControlType.kPosition);
        rightPID.setReference(rotations, ControlType.kPosition);
    }

    public Command goToPreset(ClimberPreset preset) {
        return run(() -> setPosition(preset.rot))
                .until(() -> isAtTarget(preset.rot))
                .withName("Climb " + preset.name());
    }

    public boolean isAtTarget(double targetRot) {
        double current = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2.0;
        return Math.abs(current - targetRot) < ClimberConstants.kPositionTolerance;
    }

    public boolean isSeated() {
        return seatSensor.get();
    }

    // =========================================================================
    // LEGACY / COMPATIBILITY
    // =========================================================================

    public void extend() {
        setPosition(ClimberPreset.EXTEND.rot);
    }

    public void retract() {
        setPosition(ClimberPreset.RETRACT.rot);
    }

    public void hold() {
        setPosition(ClimberPreset.HOLD.rot);
    }

    public void home() {
        setPosition(ClimberPreset.HOME.rot);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isAtTarget() {
        // Hedef bilinmiyor, basitçe hareket durdu mu kontrol edilebilir veya
        // son set edilen preset'e göre bakılabilir. Şimdilik hep false veya
        // mevcut pozisyonun bir preset'e yakınlığına bakılabilir.
        // En iyisi son hedefi saklamak.
        return false;
    }

    // Manual Voltage for Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }
}
