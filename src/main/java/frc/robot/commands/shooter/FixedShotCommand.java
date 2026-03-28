package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.TunableNumber;

import java.util.function.Supplier;

/**
 * Fixed Shooting Position Command.
 * 
 * Does NOT use field data - works entirely with TunableNumber-based fixed parameters.
 * Turret Angle, RPM and Hood Angle are adjustable from the Dashboard.
 * 
 * <h2>Turret Angle Compensation:</h2>
 * <p>
 * The turret angle stored in constants is FIELD-RELATIVE (absolute field angle).
 * During execution, the command subtracts the robot's current heading (gyro)
 * to convert to ROBOT-RELATIVE turret angle. This way the robot doesn't need
 * to be perfectly aligned - the turret compensates automatically.
 * </p>
 * 
 * <h2>Operation Logic:</h2>
 * <ol>
 * <li>Reads TunableNumber values for the specified point</li>
 * <li>Reads robot heading and compensates turret angle</li>
 * <li>Sets Turret, Hood and Flywheel to target values</li>
 * <li>Runs feeder when flywheel is ready (fire-latch)</li>
 * <li>Must be held - stops when released</li>
 * </ol>
 */
public class FixedShotCommand extends Command {

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final IntakeSubsystem intake;
    private final Supplier<Pose2d> poseSupplier;

    // Which point's parameters to use
    private final TunableNumber turretAngleTunable;
    private final TunableNumber rpmTunable;
    private final TunableNumber hoodAngleTunable;
    private final String shotName;

    // Fire latch - once started, feeder keeps running
    private boolean hasShot = false;

    // Intake deploy/retract cycle (agitation)
    private final Timer cycleTimer = new Timer();
    private boolean intakeDeployed = false;

    // Tunable: cycle time (seconds) and low RPM (same as ShootCommand)
    private static final TunableNumber intakeCycleTime = new TunableNumber("Shooter", "FixedShot/CycleTime", 1.5);
    private static final TunableNumber intakeShootRPM = new TunableNumber("Shooter", "FixedShot/IntakeRPM", 1500.0);

    /**
     * Creates a fixed shooting position command.
     * 
     * @param shooter      Shooter alt sistemi
     * @param feeder       Feeder alt sistemi
     * @param intake       Intake alt sistemi
     * @param poseSupplier Robot pose kaynağı (heading kompanzasyonu için)
     * @param index        Nokta indeksi (0-5: R1-R4/RP1-RP2 veya B1-B4/BP1-BP2)
     * @param isRed        true = Red alliance noktaları, false = Blue alliance noktaları
     */
    public FixedShotCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            IntakeSubsystem intake,
            Supplier<Pose2d> poseSupplier,
            int index,
            boolean isRed) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.poseSupplier = poseSupplier;

        if (isRed) {
            this.turretAngleTunable = ShooterConstants.RED_FIXED_TURRET[index];
            this.rpmTunable = ShooterConstants.RED_FIXED_RPM[index];
            this.hoodAngleTunable = ShooterConstants.RED_FIXED_HOOD[index];
            this.shotName = ShooterConstants.RED_FIXED_SHOT_NAMES[index];
        } else {
            this.turretAngleTunable = ShooterConstants.BLUE_FIXED_TURRET[index];
            this.rpmTunable = ShooterConstants.BLUE_FIXED_RPM[index];
            this.hoodAngleTunable = ShooterConstants.BLUE_FIXED_HOOD[index];
            this.shotName = ShooterConstants.BLUE_FIXED_SHOT_NAMES[index];
        }

        addRequirements(shooter, feeder, intake);
    }

    /**
     * Converts a field-relative turret angle to robot-relative
     * using the robot's current heading.
     * 
     * @param fieldRelativeAngleDeg Absolute field turret angle (degrees)
     * @return Robot-relative turret angle (degrees)
     */
    private double compensateForRobotHeading(double fieldRelativeAngleDeg) {
        Pose2d currentPose = poseSupplier.get();
        double robotHeadingDeg = currentPose.getRotation().getDegrees();

        // Field-relative açıdan robot heading'ini çıkar → robot-relative açı
        double robotRelativeAngle = fieldRelativeAngleDeg - robotHeadingDeg;

        // -180 ile 180 arasına normalize et
        robotRelativeAngle = MathUtil.inputModulus(robotRelativeAngle, -180.0, 180.0);

        return robotRelativeAngle;
    }

    @Override
    public void initialize() {
        hasShot = false;
        intakeDeployed = false;
        cycleTimer.restart();

        // Read values, compensate for heading, and apply
        double fieldTurretAngle = turretAngleTunable.get();
        double compensatedTurret = compensateForRobotHeading(fieldTurretAngle);
        double rpm = rpmTunable.get();
        double hoodAngle = hoodAngleTunable.get();

        shooter.setTurretAngle(compensatedTurret);
        shooter.setFlywheelRPM(rpm);
        shooter.setHoodAngle(hoodAngle);

        System.out.println("[FixedShot] " + shotName + " Started - FieldTurret=" + fieldTurretAngle
                + "deg, Compensated=" + String.format("%.1f", compensatedTurret)
                + "deg, RPM=" + rpm + ", Hood=" + hoodAngle + "deg");
    }

    @Override
    public void execute() {
        // Continuously compensate for heading (robot may rotate)
        double fieldTurretAngle = turretAngleTunable.get();
        double compensatedTurret = compensateForRobotHeading(fieldTurretAngle);

        shooter.setTurretAngle(compensatedTurret);
        shooter.setFlywheelRPM(rpmTunable.get());
        shooter.setHoodAngle(hoodAngleTunable.get());

        // Fire-latch logic
        boolean ready = shooter.isFlywheelAtTarget();

        if (ready && !hasShot) {
            hasShot = true;
        }

        // LATCH LOGIC:
        if (ready || hasShot) {
            feeder.feed();
        } else {
            feeder.stop();
        }

        // Intake agitation - slow deploy/retract cycle + low RPM roller
        intake.runRollerRPM(intakeShootRPM.get());

        if (cycleTimer.hasElapsed(intakeCycleTime.get())) {
            intakeDeployed = !intakeDeployed;
            if (intakeDeployed) {
                intake.setExtensionPosition(IntakeConstants.kExtensionDeployedCm);
            } else {
                intake.setExtensionPosition(IntakeConstants.kExtensionHalfRetractedCm);
            }
            cycleTimer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        feeder.stop();
        intake.stopRoller();
        intake.setExtensionPosition(IntakeConstants.kExtensionRetractedCm);
        cycleTimer.stop();

        System.out.println("[FixedShot] " + shotName + " Stopped");
    }

    @Override
    public boolean isFinished() {
        return false; // Must be held - stops when released
    }
}
