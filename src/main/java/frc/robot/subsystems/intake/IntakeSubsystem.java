package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotMap;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * Intake Subsystem
 * Roller: TalonFX (Kraken) — Velocity Control (RPM)
 * Pivot: SparkMax (NEO 1.2) — Position Control + Relative Encoder + Homing
 * 
 * Dişli oranı dashboard'dan ayarlanabilir.
 * Kullanıcı ÇIKIŞ açısı (derece) girer, sistem motor açısını hesaplar.
 * motor_açı = çıkış_açı * dişli_oranı
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;
    private final SparkMax extensionMotor;
    private final SparkClosedLoopController extensionPID;
    private final RelativeEncoder extensionEncoder;

    // Control Requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // =====================================================================
    // ROLLER PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerKP = new TunableNumber("Intake/Extension", "Roller kP", IntakeConstants.kRollerkP);
    private final TunableNumber rollerKI = new TunableNumber("Intake/Extension", "Roller kI", IntakeConstants.kRollerkI);
    private final TunableNumber rollerKD = new TunableNumber("Intake/Extension", "Roller kD", IntakeConstants.kRollerkD);
    private final TunableNumber rollerKV = new TunableNumber("Intake/Extension", "Roller kV", IntakeConstants.kRollerkV);

    // =====================================================================
    // ROLLER LIMITS (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerTargetRPM = new TunableNumber("Intake/Extension", "Roller Target RPM",
            IntakeConstants.kRollerTargetRPM);
    private final TunableNumber rollerMinRPM = new TunableNumber("Intake/Extension", "Roller Min RPM",
            IntakeConstants.kRollerMinRPM);
    private final TunableNumber rollerMaxRPM = new TunableNumber("Intake/Extension", "Roller Max RPM",
            IntakeConstants.kRollerMaxRPM);

    // =====================================================================
    // EXTENSION PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber extensionKP = new TunableNumber("Intake/Extension", "kP", IntakeConstants.kExtensionP);
    private final TunableNumber extensionKI = new TunableNumber("Intake/Extension", "kI", IntakeConstants.kExtensionI);
    private final TunableNumber extensionKD = new TunableNumber("Intake/Extension", "kD", IntakeConstants.kExtensionD);

    // =====================================================================
    // EXTENSION POSITIONS & LIMITS (çıkış santimetre — RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber extensionDeployedCm = new TunableNumber("Intake/Extension", "Deployed Cm",
            IntakeConstants.kExtensionDeployedCm);
    private final TunableNumber extensionRetractedCm = new TunableNumber("Intake/Extension", "Retracted Cm",
            IntakeConstants.kExtensionRetractedCm);
    private final TunableNumber extensionMinCm = new TunableNumber("Intake/Extension", "Min Cm",
            IntakeConstants.kExtensionMinCm);
    private final TunableNumber extensionMaxCm = new TunableNumber("Intake/Extension", "Max Cm",
            IntakeConstants.kExtensionMaxCm);

    // =====================================================================
    // DİŞLİ ORANI VE PİNYON ÇAPI (RIO'ya kaydedilir — Elastic'ten değiştirilebilir)
    // =====================================================================
    private final TunableNumber extensionGearRatio = new TunableNumber("Intake/Extension", "Gear Ratio",
            IntakeConstants.kExtensionGearRatio);
    private final TunableNumber extensionPinionDiameterCm = new TunableNumber("Intake/Extension", "Pinion Diameter Cm",
            IntakeConstants.kExtensionPinionDiameterCm);

    // =====================================================================
    // HOMING (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber homingVoltage = new TunableNumber("Intake/Extension", "Homing Voltage",
            IntakeConstants.kExtensionHomingVoltage);
    private final TunableNumber homingDuration = new TunableNumber("Intake/Extension", "Homing Duration",
            IntakeConstants.kExtensionHomingDurationSec);

    // =====================================================================
    // TELETUNING (Dashboard toggle — RIO'ya kaydedilir)
    // =====================================================================
    private boolean lastExtensionMotorInvertState;
    private boolean lastRollerInvertState;
    private boolean lastEnableSoftLimitsState;

    // =====================================================================
    // STATE
    // =====================================================================
    private boolean motorsEnabled = true;
    private boolean manualOverrideEnabled = false;
    private boolean isHomed = false;

    // =====================================================================
    // SIMULATION
    // =====================================================================
    private double simExtensionCm = 0.0;
    private double simRollerRPM = 0.0;
    private double lastExtensionSetpointCm = 0.0;
    private double lastRollerSetpointRPM = 0.0;

    public IntakeSubsystem() {
        // --- FIRST: Read Saved Preferences and Push to Dashboard ---
        boolean savedExtensionMotorInvert = edu.wpi.first.wpilibj.Preferences.getBoolean(
                "Intake/Extension/MotorInvert", IntakeConstants.kExtensionMotorInverted);
        boolean savedRollerInvert = edu.wpi.first.wpilibj.Preferences.getBoolean(
                "Intake/Extension/RollerInvert", IntakeConstants.kRollerInverted);
        boolean savedEnableSoftLimits = edu.wpi.first.wpilibj.Preferences.getBoolean(
                "Intake/Extension/EnableSoftLimits", true);

        SmartDashboard.setDefaultBoolean("Tuning/Intake/Extension/MotorInvert", savedExtensionMotorInvert);
        SmartDashboard.setDefaultBoolean("Tuning/Intake/Extension/RollerInvert", savedRollerInvert);
        SmartDashboard.setDefaultBoolean("Tuning/Intake/Extension/EnableSoftLimits", savedEnableSoftLimits);

        lastExtensionMotorInvertState = savedExtensionMotorInvert;
        lastRollerInvertState = savedRollerInvert;
        lastEnableSoftLimitsState = savedEnableSoftLimits;

        // --- SECOND: Initialize Motors ---
        // --- ROLLER (Kraken X60) ---
        rollerMotor = new TalonFX(RobotMap.kIntakeMotorID);
        configureRoller();

        // --- EXTENSION (NEO 1.2 — Relative Encoder) ---
        extensionMotor = new SparkMax(RobotMap.kIntakePivotMotorID, MotorType.kBrushless);
        extensionPID = extensionMotor.getClosedLoopController();
        extensionEncoder = extensionMotor.getEncoder();

        configureExtension();

        // Dashboard komutları
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Tuning/Intake/IntakeAşagı(Deploy)",
                deployCommand());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Tuning/Intake/IntakeYukarı(Retract)",
                retractCommand());

        System.out.println("[Intake] Extension: Relative Encoder, Gear Ratio=" + extensionGearRatio.get()
                + ":1, HOMING gerekli");
    }

    // =====================================================================
    // ROLLER CONFIGURATION
    // =====================================================================
    private void configureRoller() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        boolean rollerInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/RollerInvert", IntakeConstants.kRollerInverted);
        rollerConfig.MotorOutput.Inverted = rollerInvert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.Slot0.kV = rollerKV.get();
        rollerConfig.Slot0.kP = rollerKP.get();
        rollerConfig.Slot0.kI = rollerKI.get();
        rollerConfig.Slot0.kD = rollerKD.get();

        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    // =====================================================================
    // EXTENSION CONFIGURATION (Relative Encoder)
    // Encoder motor devri cinsinden okur.
    // Conversion factor: 1 motor devri = 360° (motor açısı) -> İptal edildi, 1 motor devri = 1.0 devir olarak bırakıldı
    // Çıkış (cm) = (motor_açısı_devri / gear_ratio) * (Math.PI * PinionDiameter)
    // =====================================================================
    private void configureExtension() {
        SparkMaxConfig extensionConfig = new SparkMaxConfig();

        boolean motorInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/MotorInvert",
                IntakeConstants.kExtensionMotorInverted);
        extensionConfig.inverted(motorInvert);
        extensionConfig.idleMode(IdleMode.kCoast); // Serbest hareket için Coast
        extensionConfig.smartCurrentLimit(IntakeConstants.kExtensionCurrentLimit);

        // PID — Tunable, RIO'ya kaydedilir
        extensionConfig.closedLoop.pid(extensionKP.get(), extensionKI.get(), extensionKD.get());

        // Relative Encoder: motor devri cinsinden (conversion yok, raw motor devri)
        // PID setpoint'e MOTOR DEVRİ cinsinden gönderilecek
        // Çıkış santimetre dönüşümü yazılımda yapılacak
        extensionConfig.encoder.positionConversionFactor(1.0); // 1 motor devri = 1 birim
        extensionConfig.encoder.velocityConversionFactor(1.0 / 60.0);

        // Soft Limits — MOTOR DEVRİ cinsinden
        double gearRatio = extensionGearRatio.get();
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();
        
        // Formül: motor_tur = (hedef_cm / cevre) * disli_orani
        double motorMinRev = (extensionMinCm.get() / circumferenceCm) * gearRatio;
        double motorMaxRev = (extensionMaxCm.get() / circumferenceCm) * gearRatio;
        
        boolean enableSoftLimits = SmartDashboard.getBoolean("Tuning/Intake/Extension/EnableSoftLimits", true);
        
        extensionConfig.softLimit.reverseSoftLimit((float) motorMinRev);
        extensionConfig.softLimit.reverseSoftLimitEnabled(enableSoftLimits);
        extensionConfig.softLimit.forwardSoftLimit((float) motorMaxRev);
        extensionConfig.softLimit.forwardSoftLimitEnabled(enableSoftLimits);

        // CAN Optimization: Extension Motor
        extensionConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                // We use position control so keep position at default (20ms), slow down velocity
                .primaryEncoderVelocityPeriodMs(500);

        extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("[Intake] Extension: GearRatio=" + gearRatio
                + ", Motor Limits [" + motorMinRev + ", " + motorMaxRev + "] rev"
                + ", Output Limits [" + extensionMinCm.get() + "cm, " + extensionMaxCm.get() + "cm]");
    }

    // =====================================================================
    // HOMING (hood gibi: düşük voltajla geriye sür, encoder sıfırla)
    // =====================================================================

    /**
     * Extension (rack and pinion) homing komutu.
     * Düşük voltajda geriye sürer, sonra encoder'ı 0'a sıfırlar.
     */
    public Command getHomePivotCommand() {
        return Commands.run(() -> extensionMotor.setVoltage(homingVoltage.get()), this)
                .withTimeout(homingDuration.get())
                .finallyDo(() -> {
                    extensionMotor.setVoltage(0);
                    extensionEncoder.setPosition(0.0);
                    isHomed = true;
                    System.out.println("[Intake] Extension Homed & Encoder Reset to 0 cm");
                });
    }

    public boolean isHomed() {
        return isHomed;
    }

    // =====================================================================
    // PERIODIC
    // =====================================================================
    @Override
    public void periodic() {
        checkTunableUpdates();
        checkDashboardToggles();
        logTelemetry();
    }

    private void checkTunableUpdates() {
        if (rollerKP.hasChanged() || rollerKI.hasChanged() || rollerKD.hasChanged() || rollerKV.hasChanged()) {
            configureRoller();
            System.out.println("[Intake] Roller PID güncellendi");
        }

        if (extensionKP.hasChanged() || extensionKI.hasChanged() || extensionKD.hasChanged()) {
            configureExtension();
            System.out.println("[Intake] Extension PID güncellendi");
        }

        if (extensionMinCm.hasChanged() || extensionMaxCm.hasChanged() || extensionGearRatio.hasChanged() || extensionPinionDiameterCm.hasChanged()) {
            configureExtension();
            System.out.println("[Intake] Extension limitleri/pinion/gear ratio güncellendi");
        }
    }

    private void checkDashboardToggles() {
        boolean extensionMotorInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/MotorInvert",
                IntakeConstants.kExtensionMotorInverted);
        if (extensionMotorInvert != lastExtensionMotorInvertState) {
            lastExtensionMotorInvertState = extensionMotorInvert;
            edu.wpi.first.wpilibj.Preferences.setBoolean("Intake/Extension/MotorInvert", extensionMotorInvert);
            configureExtension();
            System.out.println("[Intake] Extension Motor Invert: " + extensionMotorInvert);
        }

        boolean rollerInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/RollerInvert", IntakeConstants.kRollerInverted);
        if (rollerInvert != lastRollerInvertState) {
            lastRollerInvertState = rollerInvert;
            edu.wpi.first.wpilibj.Preferences.setBoolean("Intake/Extension/RollerInvert", rollerInvert);
            configureRoller();
            System.out.println("[Intake] Roller Invert: " + rollerInvert);
        }
        
        boolean enableSoftLimits = SmartDashboard.getBoolean("Tuning/Intake/Extension/EnableSoftLimits", true);
        if (enableSoftLimits != lastEnableSoftLimitsState) {
            lastEnableSoftLimitsState = enableSoftLimits;
            edu.wpi.first.wpilibj.Preferences.setBoolean("Intake/Extension/EnableSoftLimits", enableSoftLimits);
            configureExtension();
            System.out.println("[Intake] Extension Soft Limits Enabled: " + enableSoftLimits);
        }
    }

    private void logTelemetry() {
        double outputCm = RobotBase.isSimulation() ? simExtensionCm : getOutputCm();
        double rollerRPM = RobotBase.isSimulation() ? simRollerRPM : getRollerRPM();

        Logger.recordOutput("Tuning/Intake/ExtensionOutputCm", outputCm);
        Logger.recordOutput("Tuning/Intake/ExtensionMotorRev", getMotorRevolutions());
        Logger.recordOutput("Tuning/Intake/ExtensionSetpointCm", lastExtensionSetpointCm);
        Logger.recordOutput("Tuning/Intake/ExtensionGearRatio", extensionGearRatio.get());
        Logger.recordOutput("Tuning/Intake/ExtensionPinionDiameterCm", extensionPinionDiameterCm.get());
        Logger.recordOutput("Tuning/Intake/RollerRPM", rollerRPM);
        Logger.recordOutput("Tuning/Intake/RollerSetpointRPM", lastRollerSetpointRPM);
        Logger.recordOutput("Tuning/Intake/IsHomed", isHomed);
        Logger.recordOutput("Tuning/Intake/MotorsEnabled", motorsEnabled);
    }

    // =========================================================================
    // ROLLER COMMANDS
    // =========================================================================

    public void runRoller(double volts) {
        if (motorsEnabled && !manualOverrideEnabled) {
            rollerMotor.setControl(voltageRequest.withOutput(volts));
        }
    }

    public void runRollerRPM(double rpm) {
        if (motorsEnabled && !manualOverrideEnabled) {
            double clampedRPM = Math.max(rollerMinRPM.get(), Math.min(rpm, rollerMaxRPM.get()));
            lastRollerSetpointRPM = clampedRPM;
            rollerMotor.setControl(velocityRequest.withVelocity(clampedRPM / 60.0));
        }
    }

    public double getRollerRPM() {
        return rollerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /** Roller'ı durdurur */
    public void stopRoller() {
        rollerMotor.stopMotor();
        lastRollerSetpointRPM = 0;
    }

    public Command runRollerCommand() {
        return run(() -> runRollerRPM(rollerTargetRPM.get()))
                .finallyDo(() -> stopRoller());
    }

    public Command reverseRollerCommand() {
        return run(() -> runRollerRPM(-rollerTargetRPM.get()))
                .finallyDo(() -> stopRoller());
    }

    public Command stopRollerCommand() {
        return runOnce(() -> rollerMotor.stopMotor());
    }

    // =========================================================================
    // EXTENSION COMMANDS
    // =========================================================================

    /**
     * Extension'ı hedeflenen cm'e gönderir.
     * İçeride dişli oranı ve pinyon çapına göre motor devrine çevrilir.
     */
    public void setExtensionPosition(double targetCm) {
        double clampedCm = Math.max(extensionMinCm.get(), Math.min(targetCm, extensionMaxCm.get()));
        lastExtensionSetpointCm = clampedCm;
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();
        double motorRev = (clampedCm / circumferenceCm) * extensionGearRatio.get();
        extensionPID.setReference(motorRev, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    /** Motor encoder ham değeri (motor devri). */
    public double getMotorRevolutions() {
        return extensionEncoder.getPosition();
    }

    /** Çıkış mesafesi (cm) — motor devrinden hesaplanır. */
    public double getOutputCm() {
        double gearRatio = extensionGearRatio.get();
        if (gearRatio == 0)
            return 0;
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();
        return (extensionEncoder.getPosition() / gearRatio) * circumferenceCm;
    }

    /** Anlık uzama mesafesi (cm). */
    public double getExtensionPositionCm() {
        return getOutputCm();
    }

    public Command deployCommand() {
        return runOnce(() -> setExtensionPosition(extensionDeployedCm.get()));
    }

    public Command retractCommand() {
        return runOnce(() -> setExtensionPosition(extensionRetractedCm.get()));
    }

    // =========================================================================
    // MOTOR ENABLE/DISABLE
    // =========================================================================

    public void enableMotors() {
        motorsEnabled = true;
    }

    public void disableMotors() {
        motorsEnabled = false;
        rollerMotor.stopMotor();
        extensionMotor.stopMotor();
    }

    public boolean areMotorsEnabled() {
        return motorsEnabled;
    }

    public void enableManualOverride() {
        manualOverrideEnabled = true;
    }

    public void disableManualOverride() {
        manualOverrideEnabled = false;
    }

    public void stopAll() {
        rollerMotor.stopMotor();
        extensionMotor.stopMotor();
        lastExtensionSetpointCm = 0;
        lastRollerSetpointRPM = 0;
    }

    // =========================================================================
    // SIMULATION
    // =========================================================================
    @Override
    public void simulationPeriodic() {
        simExtensionCm += (lastExtensionSetpointCm - simExtensionCm) * 0.1;
        simRollerRPM += (lastRollerSetpointRPM - simRollerRPM) * 0.2;
    }
}