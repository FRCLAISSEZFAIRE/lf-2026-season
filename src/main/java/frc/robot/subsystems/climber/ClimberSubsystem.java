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
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * Climber Subsystem (Modernized with Tunable)
 * Uses Dual NEO 1.2 (SparkMax) with Position Control
 */
public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;

    private final DigitalInput seatSensor;

    // =====================================================================
    // TUNABLE VALUES
    // =====================================================================
    private final TunableNumber kP = new TunableNumber("Climber", "kP", ClimberConstants.kClimberP);
    private final TunableNumber kI = new TunableNumber("Climber", "kI", ClimberConstants.kClimberI);
    private final TunableNumber kD = new TunableNumber("Climber", "kD", ClimberConstants.kClimberD);

    private final TunableNumber extendPosition = new TunableNumber("Climber", "Extend Position",
            ClimberConstants.kClimbExtendPosition);
    private final TunableNumber retractPosition = new TunableNumber("Climber", "Retract Position",
            ClimberConstants.kClimbRetractPosition);
    private final TunableNumber holdPosition = new TunableNumber("Climber", "Hold Position",
            ClimberConstants.kClimbHoldPosition);
    private final TunableNumber homePosition = new TunableNumber("Climber", "Home Position",
            ClimberConstants.kHomePosition);

    // =====================================================================
    // MOTOR ENABLE/DISABLE (Motor takılı değilken test için)
    // =====================================================================
    private boolean motorsEnabled = true;

    public enum ClimberPreset {
        HOME, EXTEND, RETRACT, HOLD
    }

    public ClimberSubsystem() {
        seatSensor = new DigitalInput(ClimberConstants.kSeatSensorDIO);

        leftMotor = new SparkMax(RobotMap.kClimberLeftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.kClimberRightMotorID, MotorType.kBrushless);

        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        configureMotor(leftMotor, false);
        configureMotor(rightMotor, true);

        System.out.println("[Climber] İki NEO 1.2 ile yapılandırıldı");
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ClimberConstants.kSupplyCurrentLimit);

        // Invert
        config.inverted(inverted);

        // PID - Using tunable values
        config.closedLoop.pid(kP.get(), kI.get(), kD.get());
        config.closedLoop.outputRange(-1, 1);

        // Soft Limits
        config.softLimit.forwardSoftLimit((float) ClimberConstants.kForwardSoftLimit);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit((float) ClimberConstants.kReverseSoftLimit);
        config.softLimit.reverseSoftLimitEnabled(true);

        // Position Conversion (Default 1.0 is OK for Rotations)
        config.encoder.positionConversionFactor(1.0);
        config.encoder.velocityConversionFactor(1.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0); // Assume Home
    }

    @Override
    public void periodic() {
        checkTunableUpdates();
        logTelemetry();
    }

    private void checkTunableUpdates() {
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
            configureMotor(leftMotor, false);
            configureMotor(rightMotor, true);
            System.out.println("[Climber] PID güncellendi");
        }
    }

    private void logTelemetry() {
        Logger.recordOutput("Tuning/Climber/LeftPosition", leftMotor.getEncoder().getPosition());
        Logger.recordOutput("Tuning/Climber/RightPosition", rightMotor.getEncoder().getPosition());
        Logger.recordOutput("Tuning/Climber/IsSeated", seatSensor.get());
        Logger.recordOutput("Tuning/Climber/MotorsEnabled", motorsEnabled);
    }

    // =========================================================================
    // COMMANDS
    // =========================================================================

    public void setPosition(double rotations) {
        leftPID.setReference(rotations, ControlType.kPosition);
        rightPID.setReference(rotations, ControlType.kPosition);
    }

    public Command goToPreset(ClimberPreset preset) {
        return run(() -> setPosition(getPresetPosition(preset)))
                .until(() -> isAtTarget(getPresetPosition(preset)))
                .withName("Climb " + preset.name());
    }

    private double getPresetPosition(ClimberPreset preset) {
        switch (preset) {
            case HOME:
                return homePosition.get();
            case EXTEND:
                return extendPosition.get();
            case RETRACT:
                return retractPosition.get();
            case HOLD:
                return holdPosition.get();
            default:
                return 0;
        }
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
        setPosition(extendPosition.get());
    }

    public void retract() {
        setPosition(retractPosition.get());
    }

    public void hold() {
        setPosition(holdPosition.get());
    }

    public void home() {
        setPosition(homePosition.get());
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isAtTarget() {
        return false;
    }

    // =========================================================================
    // MOTOR ENABLE/DISABLE (Motor takılı değilken test için)
    // =========================================================================

    public void enableMotors() {
        motorsEnabled = true;
        System.out.println("[Climber] Motors ENABLED");
    }

    public void disableMotors() {
        motorsEnabled = false;
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        System.out.println("[Climber] Motors DISABLED");
    }

    public boolean areMotorsEnabled() {
        return motorsEnabled;
    }

    // Manual Voltage for Override
    public void setVoltage(double volts) {
        if (motorsEnabled) {
            leftMotor.setVoltage(volts);
            rightMotor.setVoltage(volts);
        }
    }
}
