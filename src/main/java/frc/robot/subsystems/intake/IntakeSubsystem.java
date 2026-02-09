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
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotMap;
import frc.robot.constants.VisionConstants;
import frc.robot.util.TunableNumber;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import org.littletonrobotics.junction.Logger;

/**
 * Intake Subsystem (Modernized with Tunable)
 * Roller: TalonFX (Kraken) with Velocity Control
 * Pivot: SparkMax (NEO 1.2) with Position Control + Absolute Encoder
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;
    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotPID;
    private final AbsoluteEncoder pivotAbsoluteEncoder;

    // Control Requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Vision
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kIntakeCamera);
    private boolean hasGamePiece = false;
    private double targetTx = 0.0;

    // =====================================================================
    // TUNABLE VALUES (Roller - Kraken)
    // =====================================================================
    private final TunableNumber rollerTargetRPS = new TunableNumber("Intake/Roller", "Target RPS",
            IntakeConstants.kRollerTargetRPS);
    private final TunableNumber rollerVoltage = new TunableNumber("Intake/Roller", "Voltage",
            IntakeConstants.kRollerVoltage);
    private final TunableNumber rollerKP = new TunableNumber("Intake/Roller", "kP", IntakeConstants.kRollerkP);
    private final TunableNumber rollerKV = new TunableNumber("Intake/Roller", "kV", IntakeConstants.kRollerkV);

    // =====================================================================
    // TUNABLE VALUES (Pivot - NEO with Absolute Encoder)
    // =====================================================================
    private final TunableNumber pivotDeployedRad = new TunableNumber("Intake/Pivot", "Deployed Rad",
            IntakeConstants.kPivotDeployedRad);
    private final TunableNumber pivotRetractedRad = new TunableNumber("Intake/Pivot", "Retracted Rad",
            IntakeConstants.kPivotRetractedRad);
    private final TunableNumber pivotKP = new TunableNumber("Intake/Pivot", "kP", IntakeConstants.kPivotP);
    private final TunableNumber pivotKD = new TunableNumber("Intake/Pivot", "kD", IntakeConstants.kPivotD);

    // =====================================================================
    // MANUAL OVERRIDE (Motor bağlı değilken test için)
    // =====================================================================
    private boolean motorsEnabled = true;
    private boolean manualOverrideEnabled = false;

    public IntakeSubsystem() {
        // --- ROLLER (Kraken X60) ---
        rollerMotor = new TalonFX(RobotMap.kIntakeMotorID);
        configureRoller();

        // --- PIVOT (NEO 1.2 with Absolute Encoder) ---
        pivotMotor = new SparkMax(RobotMap.kIntakePivotMotorID, MotorType.kBrushless);
        pivotPID = pivotMotor.getClosedLoopController();
        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

        configurePivot();
    }

    private void configureRoller() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        // Motor Output
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current Limit
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // PID (Slot 0) - Using tunable values
        rollerConfig.Slot0.kV = rollerKV.get();
        rollerConfig.Slot0.kP = rollerKP.get();
        rollerConfig.Slot0.kI = IntakeConstants.kRollerkI;
        rollerConfig.Slot0.kD = IntakeConstants.kRollerkD;

        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    private void configurePivot() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(IntakeConstants.kPivotCurrentLimit);

        // PID for position control
        pivotConfig.closedLoop.pid(pivotKP.get(), IntakeConstants.kPivotI, pivotKD.get());

        // Use relative encoder with conversion factor
        // If absolute encoder is connected, we'll use it for position zeroing
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI);
        pivotConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

        // Soft Limits
        pivotConfig.softLimit.forwardSoftLimit((float) IntakeConstants.kPivotMaxRad);
        pivotConfig.softLimit.forwardSoftLimitEnabled(true);
        pivotConfig.softLimit.reverseSoftLimit((float) IntakeConstants.kPivotMinRad);
        pivotConfig.softLimit.reverseSoftLimitEnabled(true);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Zero position using absolute encoder if available
        try {
            double absPosition = pivotAbsoluteEncoder.getPosition();
            pivotMotor.getEncoder().setPosition(absPosition);
            System.out.println("[Intake] Pivot zeroed from absolute encoder: " + absPosition);
        } catch (Exception e) {
            // Absolute encoder not connected, use default
            pivotMotor.getEncoder().setPosition(IntakeConstants.kPivotRetractedRad);
            System.out.println("[Intake] Absolute encoder not available, using default position");
        }
    }

    @Override
    public void periodic() {
        updateVision();
        checkTunableUpdates();
        logTelemetry();
    }

    private void checkTunableUpdates() {
        // Roller PID update
        if (rollerKP.hasChanged() || rollerKV.hasChanged()) {
            configureRoller();
            System.out.println("[Intake] Roller PID güncellendi");
        }

        // Pivot PID update
        if (pivotKP.hasChanged() || pivotKD.hasChanged()) {
            configurePivot();
            System.out.println("[Intake] Pivot PID güncellendi");
        }
    }

    private void updateVision() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            hasGamePiece = true;
            targetTx = result.getBestTarget().getYaw();
        } else {
            hasGamePiece = false;
            targetTx = 0.0;
        }
    }

    private void logTelemetry() {
        Logger.recordOutput("Tuning/Intake/PivotPosition", getPivotPosition());
        Logger.recordOutput("Tuning/Intake/RollerVelocity", rollerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Tuning/Intake/HasGamePiece", hasGamePiece);
        Logger.recordOutput("Tuning/Intake/MotorsEnabled", motorsEnabled);
        Logger.recordOutput("Tuning/Intake/ManualOverride", manualOverrideEnabled);
    }

    // =========================================================================
    // ROLLER COMMANDS
    // =========================================================================

    public void runRoller(double volts) {
        if (motorsEnabled && !manualOverrideEnabled) {
            rollerMotor.setControl(voltageRequest.withOutput(volts));
        }
    }

    public void runRollerVelocity(double rps) {
        if (motorsEnabled && !manualOverrideEnabled) {
            rollerMotor.setControl(velocityRequest.withVelocity(rps));
        }
    }

    /**
     * Runs roller using Velocity Control (Tunable RPS).
     */
    public Command runRollerCommand() {
        return run(() -> runRollerVelocity(rollerTargetRPS.get()));
    }

    public Command reverseRollerCommand() {
        return run(() -> runRoller(-rollerVoltage.get()));
    }

    public Command stopRollerCommand() {
        return runOnce(() -> rollerMotor.stopMotor());
    }

    // =========================================================================
    // PIVOT COMMANDS
    // =========================================================================

    public void setPivotPosition(double radians) {
        // Standard PID Control
        pivotPID.setReference(radians, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    public double getPivotPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public Command deployCommand() {
        return runOnce(() -> setPivotPosition(pivotDeployedRad.get()));
    }

    public Command retractCommand() {
        return runOnce(() -> setPivotPosition(pivotRetractedRad.get()));
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    public boolean seesGamePiece() {
        return hasGamePiece;
    }

    public double getAlignmentError() {
        return hasGamePiece ? targetTx : 0.0;
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    // =========================================================================
    // MOTOR ENABLE/DISABLE (Motor takılı değilken test için)
    // =========================================================================

    public void enableMotors() {
        motorsEnabled = true;
        System.out.println("[Intake] Motors ENABLED");
    }

    public void disableMotors() {
        motorsEnabled = false;
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
        System.out.println("[Intake] Motors DISABLED");
    }

    public boolean areMotorsEnabled() {
        return motorsEnabled;
    }

    public void enableManualOverride() {
        manualOverrideEnabled = true;
        System.out.println("[Intake] Manual override ENABLED");
    }

    public void disableManualOverride() {
        manualOverrideEnabled = false;
        System.out.println("[Intake] Manual override DISABLED");
    }

    public void stopAll() {
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
    }
}