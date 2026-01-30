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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotMap;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Constants;

/**
 * Intake Subsystem (Modernized)
 * Roller: TalonFX (Kraken) with Velocity Control
 * Pivot: SparkMax (NEO) with Position Control
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;
    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotPID;

    // Control Requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Vision
    private final String cameraName = VisionConstants.kLimelightName;
    private boolean hasGamePiece = false;
    private double targetTx = 0.0;

    public IntakeSubsystem() {
        // --- ROLLER (Kraken X60) ---
        rollerMotor = new TalonFX(RobotMap.kIntakeMotorID);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        // Motor Output
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Yönü kontrol et

        // Current Limit
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // PID (Slot 0)
        rollerConfig.Slot0.kV = IntakeConstants.kRollerkV;
        rollerConfig.Slot0.kP = IntakeConstants.kRollerkP;
        rollerConfig.Slot0.kI = IntakeConstants.kRollerkI;
        rollerConfig.Slot0.kD = IntakeConstants.kRollerkD;

        rollerMotor.getConfigurator().apply(rollerConfig);

        // --- PIVOT (NEO) ---
        pivotMotor = new SparkMax(RobotMap.kIntakePivotMotorID, MotorType.kBrushless);
        pivotPID = pivotMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(IntakeConstants.kPivotCurrentLimit);

        // PID
        pivotConfig.closedLoop.pid(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD);

        // Conversion: 1 Rot = 2pi Rad (Direct Drive)
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI);
        pivotConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

        pivotConfig.softLimit.forwardSoftLimit(IntakeConstants.kPivotMaxRad);
        pivotConfig.softLimit.forwardSoftLimitEnabled(true);
        pivotConfig.softLimit.reverseSoftLimit(IntakeConstants.kPivotMinRad);
        pivotConfig.softLimit.reverseSoftLimitEnabled(true);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Başlangıçta encoder'ı sıfırla (Retracted varsayımı)
        pivotMotor.getEncoder().setPosition(IntakeConstants.kPivotRetractedRad);
    }

    @Override
    public void periodic() {
        updateVision();
        logTelemetry();
    }

    private void updateVision() {
        if (LimelightHelpers.getTV(cameraName)) {
            hasGamePiece = true;
            targetTx = LimelightHelpers.getTX(cameraName);
        } else {
            hasGamePiece = false;
            targetTx = 0.0;
        }
    }

    private void logTelemetry() {
        if (Constants.tuningMode) {
            org.littletonrobotics.junction.Logger.recordOutput("Intake/PivotPosition", getPivotPosition());
            org.littletonrobotics.junction.Logger.recordOutput("Intake/RollerVelocity",
                    rollerMotor.getVelocity().getValueAsDouble());
            org.littletonrobotics.junction.Logger.recordOutput("Intake/HasGamePiece", hasGamePiece);
        }
    }

    // =========================================================================
    // ROLLER COMMANDS
    // =========================================================================

    public void runRoller(double volts) {
        // Legacy support: Run on voltage
        rollerMotor.setControl(voltageRequest.withOutput(volts));
    }

    public void runRollerVelocity(double rps) {
        rollerMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /**
     * Runs roller using Velocity Control (Target RPM/RPS).
     */
    public Command runRollerCommand() {
        return run(() -> runRollerVelocity(IntakeConstants.kRollerTargetRPS));
    }

    public Command reverseRollerCommand() {
        return run(() -> runRoller(-IntakeConstants.kRollerVoltage)); // Ters voltaj (kusma genelde max voltaj olur)
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
        return runOnce(() -> setPivotPosition(IntakeConstants.kPivotDeployedRad));
    }

    public Command retractCommand() {
        return runOnce(() -> setPivotPosition(IntakeConstants.kPivotRetractedRad));
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
}