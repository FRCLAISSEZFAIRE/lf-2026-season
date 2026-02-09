package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Controller binding'lerini yöneten sınıf.
 */
public class ControllerBindings {

        private final CommandXboxController driverController;
        private final CommandXboxController operatorController;

        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ClimberSubsystem climberSubsystem;
        private final LEDSubsystem ledSubsystem;

        // Vision Aiming PID Tunable Numbers
        private final TunableNumber visionkP = new TunableNumber("Vision", "FuelAimkP", 0.045);
        private final TunableNumber visionkI = new TunableNumber("Vision", "FuelAimkI", 0.0);
        private final TunableNumber visionkD = new TunableNumber("Vision", "FuelAimkD", 0.001);

        // PID Controller instance
        private final PIDController fuelAimPID = new PIDController(visionkP.get(), visionkI.get(), visionkD.get());

        public ControllerBindings(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        DriveSubsystem driveSubsystem,
                        VisionSubsystem visionSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        FeederSubsystem feederSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        ClimberSubsystem climberSubsystem,
                        LEDSubsystem ledSubsystem) {

                this.driverController = driverController;
                this.operatorController = operatorController;
                this.driveSubsystem = driveSubsystem;
                this.visionSubsystem = visionSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.feederSubsystem = feederSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.climberSubsystem = climberSubsystem;
                this.ledSubsystem = ledSubsystem;

                fuelAimPID.setTolerance(1.0);
        }

        /** Tüm binding'leri yapılandır */
        public void configureAll() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        // =========================================================================
        // DRIVER BINDINGS
        // =========================================================================
        private void configureDriverBindings() {

                // [A Tuşu] AUTO-CENTER ON NOTE (Limelight 3A)
                driverController.a().whileTrue(
                                Commands.run(() -> {
                                        // PID katsayılarını güncelle (Dashboard'dan değişirse)
                                        if (visionkP.hasChanged() || visionkI.hasChanged() || visionkD.hasChanged()) {
                                                fuelAimPID.setPID(visionkP.get(), visionkI.get(), visionkD.get());
                                        }

                                        double xVelocity = -MathUtil.applyDeadband(driverController.getLeftY(), 0.1)
                                                        * 4.5;
                                        double yVelocity = -MathUtil.applyDeadband(driverController.getLeftX(), 0.1)
                                                        * 4.5;
                                        double rotVelocity;

                                        if (visionSubsystem.hasFuel()) {
                                                double tx = visionSubsystem.getFuelYaw();
                                                double pidOutput = fuelAimPID.calculate(tx, 0.0);
                                                rotVelocity = MathUtil.clamp(pidOutput, -1.0, 1.0) * 4.0;
                                        } else {
                                                rotVelocity = -MathUtil.applyDeadband(driverController.getRightX(), 0.1)
                                                                * 4.5;
                                        }

                                        driveSubsystem.drive(
                                                        new Translation2d(xVelocity, yVelocity),
                                                        rotVelocity,
                                                        true);

                                }, driveSubsystem));

                // [Y Tuşu] TOGGLE AUTO-AIM
                driverController.y().onTrue(Commands.runOnce(() -> {
                        if (shooterSubsystem.isAutoAimActive()) {
                                shooterSubsystem.disableAutoAim();
                        } else {
                                shooterSubsystem.enableAutoAim();
                        }
                }));

                // [RIGHT BUMPER] POSE-BASED SHOOTING
                // Calculates RPM, Hood, Turret from Robot Pose
                driverController.rightBumper().whileTrue(
                                new frc.robot.commands.shooter.ShootCommand(
                                                shooterSubsystem,
                                                feederSubsystem,
                                                driveSubsystem::getPose));
        }

        // =========================================================================
        // OPERATOR BINDINGS
        // =========================================================================
        private void configureOperatorBindings() {

                // [A Tuşu] AUTOMATIC SHOOT
                operatorController.a().whileTrue(
                                Commands.run(() -> shooterSubsystem.setFlywheelRPM(ShooterConstants.kFarFlywheelRPM),
                                                shooterSubsystem)
                                                .alongWith(
                                                                Commands.waitUntil(shooterSubsystem::isFlywheelAtTarget)
                                                                                .andThen(Commands.run(
                                                                                                feederSubsystem::feed,
                                                                                                feederSubsystem)))
                                                .beforeStarting(() -> ledSubsystem.setShooting(true))
                                                .finallyDo(() -> {
                                                        shooterSubsystem.stopShooter();
                                                        feederSubsystem.stop();
                                                        ledSubsystem.setShooting(false);
                                                }));

                // [B Tuşu] REVERSE / UNJAM
                operatorController.b().whileTrue(
                                Commands.parallel(
                                                Commands.run(() -> shooterSubsystem
                                                                .setFlywheelRPM(-ShooterConstants.kFarFlywheelRPM),
                                                                shooterSubsystem),
                                                Commands.run(feederSubsystem::reverse, feederSubsystem)))
                                .onFalse(
                                                Commands.runOnce(() -> {
                                                        shooterSubsystem.stopShooter();
                                                        feederSubsystem.stop();
                                                }, shooterSubsystem, feederSubsystem));

                // [X Tuşu] EMERGENCY STOP
                operatorController.x().onTrue(
                                Commands.runOnce(() -> {
                                        shooterSubsystem.stopShooter();
                                        feederSubsystem.stop();
                                }, shooterSubsystem, feederSubsystem));

                // [POV Sol] Aim Offset -1.0 derece (Sağa kaydır)
                operatorController.povLeft().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(-1.0)));

                // [POV Sağ] Aim Offset +1.0 derece (Sola kaydır)
                operatorController.povRight().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(1.0)));
        }
}
