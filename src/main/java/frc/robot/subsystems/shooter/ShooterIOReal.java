package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.ShooterConstants;

/**
 * Shooter için gerçek donanım IO implementasyonu.
 * Flywheel (TalonFX), Turret (TalonFX), Hood (SparkMax + AbsoluteEncoder)
 */
public class ShooterIOReal implements ShooterIO {

    private final TalonFX flywheelMotor;
    private final TalonFX turretMotor;
    private final SparkMax hoodMotor;
    private final SparkMaxConfig hoodConfig;
    private final SparkAbsoluteEncoder hoodEncoder;
    private final SparkClosedLoopController hoodController;
    private final DigitalInput mz80Sensor;

    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOReal(int flywheelID, int turretID, int hoodID) {
        flywheelMotor = new TalonFX(flywheelID);
        turretMotor = new TalonFX(turretID);
        mz80Sensor = new DigitalInput(MechanismConstants.kShooterMZ80Port);

        // Hood motor (SparkMax + AbsoluteEncoder)
        hoodMotor = new SparkMax(hoodID, MotorType.kBrushless);
        hoodConfig = new SparkMaxConfig();

        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.closedLoop.pid(
                ShooterConstants.kHoodP,
                ShooterConstants.kHoodI,
                ShooterConstants.kHoodD);
        hoodConfig.absoluteEncoder.positionConversionFactor(360.0);
        hoodConfig.closedLoop.positionWrappingEnabled(false);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodEncoder = hoodMotor.getAbsoluteEncoder();
        hoodController = hoodMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel
        inputs.flywheelVelocityRadPerSec = flywheelMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();

        // Turret
        inputs.turretAbsolutePositionRad = turretMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
        inputs.turretAppliedVolts = turretMotor.getMotorVoltage().getValueAsDouble();

        // Hood
        inputs.hoodPositionDegrees = hoodEncoder.getPosition();
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();

        // MZ80 Sensör (active-low)
        inputs.shooterSensorTriggered = !mz80Sensor.get();
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        hoodController.setReference(angleDegrees, ControlType.kPosition);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }
}
