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
    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotPID;
    private final RelativeEncoder pivotEncoder;

    // Control Requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // =====================================================================
    // ROLLER PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerKP = new TunableNumber("Intake/Roller", "kP", IntakeConstants.kRollerkP);
    private final TunableNumber rollerKI = new TunableNumber("Intake/Roller", "kI", IntakeConstants.kRollerkI);
    private final TunableNumber rollerKD = new TunableNumber("Intake/Roller", "kD", IntakeConstants.kRollerkD);
    private final TunableNumber rollerKV = new TunableNumber("Intake/Roller", "kV", IntakeConstants.kRollerkV);

    // =====================================================================
    // ROLLER LIMITS (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerTargetRPM = new TunableNumber("Intake/Roller", "Target RPM",
            IntakeConstants.kRollerTargetRPM);
    private final TunableNumber rollerMinRPM = new TunableNumber("Intake/Roller", "Min RPM",
            IntakeConstants.kRollerMinRPM);
    private final TunableNumber rollerMaxRPM = new TunableNumber("Intake/Roller", "Max RPM",
            IntakeConstants.kRollerMaxRPM);
    private final TunableNumber rollerVoltage = new TunableNumber("Intake/Roller", "Voltage",
            IntakeConstants.kRollerVoltage);

    // =====================================================================
    // PIVOT PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber pivotKP = new TunableNumber("Intake/Pivot", "kP", IntakeConstants.kPivotP);
    private final TunableNumber pivotKI = new TunableNumber("Intake/Pivot", "kI", IntakeConstants.kPivotI);
    private final TunableNumber pivotKD = new TunableNumber("Intake/Pivot", "kD", IntakeConstants.kPivotD);

    // =====================================================================
    // PIVOT POSITIONS & LIMITS (çıkış derece — RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber pivotDeployedDeg = new TunableNumber("Intake/Pivot", "Deployed Deg",
            IntakeConstants.kPivotDeployedDeg);
    private final TunableNumber pivotRetractedDeg = new TunableNumber("Intake/Pivot", "Retracted Deg",
            IntakeConstants.kPivotRetractedDeg);
    private final TunableNumber pivotMinDeg = new TunableNumber("Intake/Pivot", "Min Deg",
            IntakeConstants.kPivotMinDeg);
    private final TunableNumber pivotMaxDeg = new TunableNumber("Intake/Pivot", "Max Deg",
            IntakeConstants.kPivotMaxDeg);

    // =====================================================================
    // DİŞLİ ORANI (RIO'ya kaydedilir — Elastic'ten değiştirilebilir)
    // =====================================================================
    private final TunableNumber pivotGearRatio = new TunableNumber("Intake/Pivot", "Gear Ratio",
            IntakeConstants.kPivotGearRatio);

    // =====================================================================
    // HOMING (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber homingVoltage = new TunableNumber("Intake/Pivot", "Homing Voltage",
            IntakeConstants.kPivotHomingVoltage);
    private final TunableNumber homingDuration = new TunableNumber("Intake/Pivot", "Homing Duration",
            IntakeConstants.kPivotHomingDurationSec);

    // =====================================================================
    // MOTOR INVERT (Dashboard toggle — RIO'ya kaydedilir)
    // =====================================================================
    private boolean lastPivotMotorInvertState;
    private boolean lastRollerInvertState;

    // =====================================================================
    // STATE
    // =====================================================================
    private boolean motorsEnabled = true;
    private boolean manualOverrideEnabled = false;
    private boolean isHomed = false;

    // =====================================================================
    // SIMULATION
    // =====================================================================
    private double simPivotDeg = 0.0;
    private double simRollerRPM = 0.0;
    private double lastPivotSetpointDeg = 0.0;
    private double lastRollerSetpointRPM = 0.0;

    public IntakeSubsystem() {
        // --- FIRST: Read Saved Preferences and Push to Dashboard ---
        boolean savedPivotMotorInvert = edu.wpi.first.wpilibj.Preferences.getBoolean(
                "Intake/Pivot/MotorInvert", IntakeConstants.kPivotMotorInverted);
        boolean savedRollerInvert = edu.wpi.first.wpilibj.Preferences.getBoolean(
                "Intake/Roller/Invert", IntakeConstants.kRollerInverted);

        SmartDashboard.setDefaultBoolean("Intake/Pivot/MotorInvert", savedPivotMotorInvert);
        SmartDashboard.setDefaultBoolean("Intake/Roller/Invert", savedRollerInvert);

        lastPivotMotorInvertState = savedPivotMotorInvert;
        lastRollerInvertState = savedRollerInvert;

        // --- SECOND: Initialize Motors ---
        // --- ROLLER (Kraken X60) ---
        rollerMotor = new TalonFX(RobotMap.kIntakeMotorID);
        configureRoller();

        // --- PIVOT (NEO 1.2 — Relative Encoder) ---
        pivotMotor = new SparkMax(RobotMap.kIntakePivotMotorID, MotorType.kBrushless);
        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();

        configurePivot();

        // Dashboard komutları
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Tuning/Intake/IntakeAşagı(Deploy)",
                deployCommand());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Tuning/Intake/IntakeYukarı(Retract)",
                retractCommand());

        System.out.println("[Intake] Pivot: Relative Encoder, Gear Ratio=" + pivotGearRatio.get()
                + ":1, HOMING gerekli");
    }

    // =====================================================================
    // ROLLER CONFIGURATION
    // =====================================================================
    private void configureRoller() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        boolean rollerInvert = SmartDashboard.getBoolean("Intake/Roller/Invert", IntakeConstants.kRollerInverted);
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
    // PIVOT CONFIGURATION (Relative Encoder)
    // Encoder motor devri cinsinden okur.
    // Conversion factor: 1 motor devri = 360° (motor açısı)
    // Çıkış açısı = motor_açısı / gear_ratio
    // =====================================================================
    private void configurePivot() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        boolean motorInvert = SmartDashboard.getBoolean("Intake/Pivot/MotorInvert",
                IntakeConstants.kPivotMotorInverted);
        pivotConfig.inverted(motorInvert);
        pivotConfig.idleMode(IdleMode.kCoast); // Serbest düşüş için Coast
        pivotConfig.smartCurrentLimit(IntakeConstants.kPivotCurrentLimit);

        // PID — Tunable, RIO'ya kaydedilir
        pivotConfig.closedLoop.pid(pivotKP.get(), pivotKI.get(), pivotKD.get());

        // Relative Encoder: motor devri cinsinden (conversion yok, raw motor devri)
        // PID setpoint'e MOTOR DEVRİ cinsinden gönderilecek
        // Çıkış açısı dönüşümü yazılımda yapılacak
        pivotConfig.encoder.positionConversionFactor(1.0); // 1 motor devri = 1 birim
        pivotConfig.encoder.velocityConversionFactor(1.0 / 60.0);

        // Soft Limits — MOTOR DEVRİ cinsinden
        double gearRatio = pivotGearRatio.get();
        double motorMinRev = pivotMinDeg.get() / 360.0 * gearRatio;
        double motorMaxRev = pivotMaxDeg.get() / 360.0 * gearRatio;
        pivotConfig.softLimit.reverseSoftLimit((float) motorMinRev);
        pivotConfig.softLimit.reverseSoftLimitEnabled(true);
        pivotConfig.softLimit.forwardSoftLimit((float) motorMaxRev);
        pivotConfig.softLimit.forwardSoftLimitEnabled(true);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("[Intake] Pivot: GearRatio=" + gearRatio
                + ", Motor Limits [" + motorMinRev + ", " + motorMaxRev + "] rev"
                + ", Output Limits [" + pivotMinDeg.get() + "°, " + pivotMaxDeg.get() + "°]");
    }

    // =====================================================================
    // HOMING (hood gibi: düşük voltajla geriye sür, encoder sıfırla)
    // =====================================================================

    /**
     * Pivot homing komutu.
     * Düşük voltajda geriye sürer, sonra encoder'ı 0'a sıfırlar.
     */
    public Command getHomePivotCommand() {
        return Commands.run(() -> pivotMotor.setVoltage(homingVoltage.get()), this)
                .withTimeout(homingDuration.get())
                .finallyDo(() -> {
                    pivotMotor.setVoltage(0);
                    pivotEncoder.setPosition(0.0);
                    isHomed = true;
                    System.out.println("[Intake] Pivot Homed & Encoder Reset to 0°");
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
        checkInvertChanges();
        logTelemetry();
    }

    private void checkTunableUpdates() {
        if (rollerKP.hasChanged() || rollerKI.hasChanged() || rollerKD.hasChanged() || rollerKV.hasChanged()) {
            configureRoller();
            System.out.println("[Intake] Roller PID güncellendi");
        }

        if (pivotKP.hasChanged() || pivotKI.hasChanged() || pivotKD.hasChanged()) {
            configurePivot();
            System.out.println("[Intake] Pivot PID güncellendi");
        }

        if (pivotMinDeg.hasChanged() || pivotMaxDeg.hasChanged() || pivotGearRatio.hasChanged()) {
            configurePivot();
            System.out.println("[Intake] Pivot limitleri/gear ratio güncellendi");
        }
    }

    private void checkInvertChanges() {
        boolean pivotMotorInvert = SmartDashboard.getBoolean("Intake/Pivot/MotorInvert",
                IntakeConstants.kPivotMotorInverted);
        if (pivotMotorInvert != lastPivotMotorInvertState) {
            lastPivotMotorInvertState = pivotMotorInvert;
            edu.wpi.first.wpilibj.Preferences.setBoolean("Intake/Pivot/MotorInvert", pivotMotorInvert);
            configurePivot();
            System.out.println("[Intake] Pivot Motor Invert: " + pivotMotorInvert);
        }

        boolean rollerInvert = SmartDashboard.getBoolean("Intake/Roller/Invert", IntakeConstants.kRollerInverted);
        if (rollerInvert != lastRollerInvertState) {
            lastRollerInvertState = rollerInvert;
            edu.wpi.first.wpilibj.Preferences.setBoolean("Intake/Roller/Invert", rollerInvert);
            configureRoller();
            System.out.println("[Intake] Roller Invert: " + rollerInvert);
        }
    }

    private void logTelemetry() {
        double outputDeg = RobotBase.isSimulation() ? simPivotDeg : getOutputDegrees();
        double rollerRPM = RobotBase.isSimulation() ? simRollerRPM : getRollerRPM();

        Logger.recordOutput("Tuning/Intake/PivotOutputDeg", outputDeg);
        Logger.recordOutput("Tuning/Intake/PivotMotorRev", getMotorRevolutions());
        Logger.recordOutput("Tuning/Intake/PivotSetpointDeg", lastPivotSetpointDeg);
        Logger.recordOutput("Tuning/Intake/PivotGearRatio", pivotGearRatio.get());
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

    public Command runRollerCommand() {
        return run(() -> runRollerRPM(rollerTargetRPM.get()))
                .finallyDo(() -> {
                    rollerMotor.stopMotor();
                    lastRollerSetpointRPM = 0;
                });
    }

    public Command reverseRollerCommand() {
        return run(() -> runRoller(-rollerVoltage.get()))
                .finallyDo(() -> {
                    rollerMotor.stopMotor();
                    lastRollerSetpointRPM = 0;
                });
    }

    public Command stopRollerCommand() {
        return runOnce(() -> rollerMotor.stopMotor());
    }

    // =========================================================================
    // PIVOT COMMANDS
    // Kullanıcı ÇIKIŞ DERECESİ girer → motor devrine çevrilir
    // motor_rev = output_deg / 360.0 * gear_ratio
    // =========================================================================

    /**
     * Pivot'u çıkış açısına gönderir (derece).
     * İçeride dişli oranına göre motor devrine çevrilir.
     */
    public void setPivotPosition(double outputDegrees) {
        double clampedDeg = Math.max(pivotMinDeg.get(), Math.min(outputDegrees, pivotMaxDeg.get()));
        lastPivotSetpointDeg = clampedDeg;
        double motorRev = clampedDeg / 360.0 * pivotGearRatio.get();
        pivotPID.setReference(motorRev, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    /** Motor encoder ham değeri (motor devri). */
    public double getMotorRevolutions() {
        return pivotEncoder.getPosition();
    }

    /** Çıkış açısı (derece) — motor devrinden hesaplanır. */
    public double getOutputDegrees() {
        double gearRatio = pivotGearRatio.get();
        if (gearRatio == 0)
            return 0;
        return pivotEncoder.getPosition() / gearRatio * 360.0;
    }

    /** Anlık çıkış açısı (getPivotPosition alias). */
    public double getPivotPosition() {
        return getOutputDegrees();
    }

    public Command deployCommand() {
        return runOnce(() -> setPivotPosition(pivotDeployedDeg.get()));
    }

    public Command retractCommand() {
        return runOnce(() -> setPivotPosition(pivotRetractedDeg.get()));
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    public boolean seesGamePiece() {
        return false;
    }

    public double getAlignmentError() {
        return 0.0;
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
        pivotMotor.stopMotor();
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
        pivotMotor.stopMotor();
        lastPivotSetpointDeg = 0;
        lastRollerSetpointRPM = 0;
    }

    // =========================================================================
    // SIMULATION
    // =========================================================================
    @Override
    public void simulationPeriodic() {
        simPivotDeg += (lastPivotSetpointDeg - simPivotDeg) * 0.1;
        simRollerRPM += (lastRollerSetpointRPM - simRollerRPM) * 0.2;
    }
}