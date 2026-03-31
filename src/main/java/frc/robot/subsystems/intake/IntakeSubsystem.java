package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // =====================================================================
    // ROLLER PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerKP = new TunableNumber("Tuning/Intake/Extension", "Roller kP", IntakeConstants.kRollerkP);
    private final TunableNumber rollerKI = new TunableNumber("Tuning/Intake/Extension", "Roller kI", IntakeConstants.kRollerkI);
    private final TunableNumber rollerKD = new TunableNumber("Tuning/Intake/Extension", "Roller kD", IntakeConstants.kRollerkD);
    private final TunableNumber rollerKV = new TunableNumber("Tuning/Intake/Extension", "Roller kV", IntakeConstants.kRollerkV);

    // =====================================================================
    // ROLLER LIMITS (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber rollerTargetRPM = new TunableNumber("Tuning/Intake/Extension", "Roller Target RPM",
            IntakeConstants.kRollerTargetRPM);
    private final TunableNumber rollerMinRPM = new TunableNumber("Tuning/Intake/Extension", "Roller Min RPM",
            IntakeConstants.kRollerMinRPM);
    private final TunableNumber rollerMaxRPM = new TunableNumber("Tuning/Intake/Extension", "Roller Max RPM",
            IntakeConstants.kRollerMaxRPM);

    // =====================================================================
    // EXTENSION PID (RIO'ya kaydedilir)
    // =====================================================================
    private final TunableNumber extensionKP = new TunableNumber("Tuning/Intake/Extension", "kP", IntakeConstants.kExtensionP);
    private final TunableNumber extensionKI = new TunableNumber("Tuning/Intake/Extension", "kI", IntakeConstants.kExtensionI);
    private final TunableNumber extensionKD = new TunableNumber("Tuning/Intake/Extension", "kD", IntakeConstants.kExtensionD);

    // =====================================================================
    // EXTENSION POSITIONS & LIMITS (çıkış santimetre)
    // =====================================================================
    private final TunableNumber extensionDeployedCm = new TunableNumber("Tuning/Intake/Extension", "Deployed Cm",
            IntakeConstants.kExtensionDeployedCm);
    private final TunableNumber extensionRetractedCm = new TunableNumber("Tuning/Intake/Extension", "Retracted Cm",
            IntakeConstants.kExtensionRetractedCm);
    private final TunableNumber extensionMinCm = new TunableNumber("Tuning/Intake/Extension", "Min Cm",
            IntakeConstants.kExtensionMinCm);
    private final TunableNumber extensionMaxCm = new TunableNumber("Tuning/Intake/Extension", "Max Cm",
            IntakeConstants.kExtensionMaxCm);

    // =====================================================================
    // DİŞLİ ORANI VE PİNYON ÇAPI
    // =====================================================================
    private final TunableNumber extensionGearRatio = new TunableNumber("Tuning/Intake/Extension", "Gear Ratio",
            IntakeConstants.kExtensionGearRatio);
    private final TunableNumber extensionPinionDiameterCm = new TunableNumber("Tuning/Intake/Extension", "Pinion Diameter Cm",
            IntakeConstants.kExtensionPinionDiameterCm);

    // =====================================================================
    // HOMING
    // =====================================================================
    private final TunableNumber homingVoltage = new TunableNumber("Tuning/Intake/Extension", "Homing Voltage",
            IntakeConstants.kExtensionHomingVoltage);
    private final TunableNumber homingDuration = new TunableNumber("Tuning/Intake/Extension", "Homing Duration",
            IntakeConstants.kExtensionHomingDurationSec);

    // =====================================================================
    // TELETUNING
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
    private double simExtensionCm = 0.0;
    private double simRollerRPM = 0.0;
    private double lastExtensionSetpointCm = 0.0;
    private double lastRollerSetpointRPM = 0.0;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;

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

        configureRoller();
        configureExtension();

        SmartDashboard.putData("Tuning/Intake/IntakeAşagı(Deploy)", deployCommand());
        SmartDashboard.putData("Tuning/Intake/IntakeYukarı(Retract)", retractCommand());

        System.out.println("[Intake] Extension: Gear Ratio=" + extensionGearRatio.get() + ":1, HONING gerekli (IO Abstraction)");
    }

    private void configureRoller() {
        boolean rollerInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/RollerInvert", IntakeConstants.kRollerInverted);
        io.configRoller(rollerKP.get(), rollerKI.get(), rollerKD.get(), rollerKV.get(), rollerInvert);
    }

    private void configureExtension() {
        boolean motorInvert = SmartDashboard.getBoolean("Tuning/Intake/Extension/MotorInvert",
                IntakeConstants.kExtensionMotorInverted);
        boolean enableSoftLimits = SmartDashboard.getBoolean("Tuning/Intake/Extension/EnableSoftLimits", true);

        double gearRatio = extensionGearRatio.get();
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();

        double motorMinRev = (extensionMinCm.get() / circumferenceCm) * gearRatio;
        double motorMaxRev = (extensionMaxCm.get() / circumferenceCm) * gearRatio;

        io.configExtension(extensionKP.get(), extensionKI.get(), extensionKD.get(), motorMinRev, motorMaxRev, enableSoftLimits, motorInvert);
    }

    public Command getHomePivotCommand() {
        return Commands.run(() -> io.setExtensionVoltage(homingVoltage.get()), this)
                .withTimeout(homingDuration.get())
                .finallyDo(() -> {
                    io.setExtensionVoltage(0);
                    io.resetExtensionEncoder(0.0);
                    isHomed = true;
                    System.out.println("[Intake] Extension Homed & Encoder Reset to 0 cm");
                });
    }

    public boolean isHomed() {
        return isHomed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

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

    public void runRoller(double volts) {
        if (motorsEnabled && !manualOverrideEnabled) {
            io.setRollerVoltage(volts);
        }
    }

    public void runRollerRPM(double rpm) {
        if (motorsEnabled && !manualOverrideEnabled) {
            double clampedRPM = Math.max(rollerMinRPM.get(), Math.min(rpm, rollerMaxRPM.get()));
            lastRollerSetpointRPM = clampedRPM;
            io.setRollerVelocity(clampedRPM);
        }
    }

    public double getRollerRPM() {
        return inputs.rollerVelocityRPM;
    }

    public void stopRoller() {
        io.stopRoller();
        lastRollerSetpointRPM = 0;
    }

    public Command runRollerCommand() {
        return run(() -> runRollerRPM(rollerTargetRPM.get()))
                .finallyDo(this::stopRoller);
    }

    public Command reverseRollerCommand() {
        return run(() -> runRollerRPM(-rollerTargetRPM.get()))
                .finallyDo(this::stopRoller);
    }

    public Command stopRollerCommand() {
        return runOnce(this::stopRoller);
    }

    public void setExtensionPosition(double targetCm) {
        double clampedCm = Math.max(extensionMinCm.get(), Math.min(targetCm, extensionMaxCm.get()));
        lastExtensionSetpointCm = clampedCm;
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();
        double motorRev = (clampedCm / circumferenceCm) * extensionGearRatio.get();
        io.setExtensionPositionRevs(motorRev);
    }

    public double getMotorRevolutions() {
        return inputs.extensionPositionRevs;
    }

    public double getOutputCm() {
        double gearRatio = extensionGearRatio.get();
        if (gearRatio == 0)
            return 0;
        double circumferenceCm = Math.PI * extensionPinionDiameterCm.get();
        return (inputs.extensionPositionRevs / gearRatio) * circumferenceCm;
    }

    public double getExtensionPositionCm() {
        return getOutputCm();
    }

    public void deploy() {
        setExtensionPosition(extensionDeployedCm.get());
    }

    public void retract() {
        setExtensionPosition(extensionRetractedCm.get());
    }

    public Command deployCommand() {
        return runOnce(this::deploy);
    }

    public Command retractCommand() {
        return runOnce(this::retract);
    }

    public void enableMotors() {
        motorsEnabled = true;
    }

    public void disableMotors() {
        motorsEnabled = false;
        io.stopRoller();
        io.stopExtension();
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
        io.stopRoller();
        io.stopExtension();
        lastExtensionSetpointCm = 0;
        lastRollerSetpointRPM = 0;
    }

    @Override
    public void simulationPeriodic() {
        simExtensionCm += (lastExtensionSetpointCm - simExtensionCm) * 0.1;
        simRollerRPM += (lastRollerSetpointRPM - simRollerRPM) * 0.2;
    }
}