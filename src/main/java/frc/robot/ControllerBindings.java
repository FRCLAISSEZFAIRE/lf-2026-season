package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;

import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.shooter.ShootCommand;

/**
 * Controller binding'lerini yöneten sınıf.
 * 
 * <p>
 * RobotContainer'ı temiz tutmak için tüm buton atamaları bu sınıfta
 * toplanmıştır.
 * </p>
 * 
 * <h2>Controller Dağılımı:</h2>
 * <ul>
 * <li><b>Driver Controller (Port 0):</b> Sürüş, Gyro, X-Stance</li>
 * <li><b>Operator Controller (Port 1):</b> Tüm mekanizmalar</li>
 * </ul>
 * 
 * <h2>Operator Buton Grupları:</h2>
 * <ul>
 * <li><b>Intake/Feeder:</b> Sağ/Sol Tetik</li>
 * <li><b>Shooter:</b> Sağ Bumper</li>
 * <li><b>Lift:</b> D-Pad (Elevator) + Sol Bumper + Sağ Stick (Climb)</li>
 * <li><b>Acil:</b> Start butonu</li>
 * </ul>
 * 
 * @author FRC Team
 * @see RobotContainer
 */
public class ControllerBindings {

        private final CommandXboxController driverController;
        private final CommandXboxController operatorController;

        // Subsystem referansları
        private final DriveSubsystem driveSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final LiftSubsystem liftSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final LEDSubsystem ledSubsystem;

        public ControllerBindings(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem,
                        FeederSubsystem feederSubsystem,
                        LEDSubsystem ledSubsystem) {

                this.driverController = driverController;
                this.operatorController = operatorController;
                this.driveSubsystem = driveSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.liftSubsystem = liftSubsystem;
                this.feederSubsystem = feederSubsystem;
                this.ledSubsystem = ledSubsystem;
        }

        /** Tüm binding'leri yapılandır */
        public void configureAll() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        // ==================== DRIVER BINDINGS ====================
        private void configureDriverBindings() {
                // Driver Controller Temizlendi.
                // Yeni bindingler RobotContainer içinde tanımlanıyor:
                // POV: Skor Seçimi
                // X: Pathfinding
        }

        private void configureOperatorBindings() {
                configureIntakeBindings();
                configureShooterBindings();
                configureLiftBindings();
                configureEmergencyBindings();
        }

        // --- INTAKE / FEEDER ---
        private void configureIntakeBindings() {
                // Right Trigger: Intake + Feeder (alma)
                operatorController.rightTrigger(0.5)
                                .whileTrue(Commands.parallel(
                                                new RunIntakeCommand(intakeSubsystem, 12.0),
                                                Commands.run(() -> feederSubsystem.feed(), feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIntaking())))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(feederSubsystem::stop, feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));

                // Left Trigger: Intake + Feeder Ters (çıkarma)
                operatorController.leftTrigger(0.5)
                                .whileTrue(Commands.parallel(
                                                new RunIntakeCommand(intakeSubsystem, -12.0),
                                                Commands.run(() -> feederSubsystem.reverse(), feederSubsystem)))
                                .onFalse(Commands.runOnce(feederSubsystem::stop, feederSubsystem));

                // Back Button: Auto-Intake (kamerayı takip et ve al)
                operatorController.back()
                                .whileTrue(new AutoIntakeCommand(intakeSubsystem, driveSubsystem, driverController))
                                .onFalse(Commands.runOnce(() -> ledSubsystem.setIdle()));
        }

        // --- SHOOTER ---
        private void configureShooterBindings() {
                // Right Bumper: Atış
                operatorController.rightBumper()
                                .whileTrue(Commands.sequence(
                                                Commands.runOnce(() -> ledSubsystem
                                                                .setState(LEDSubsystem.LEDState.SHOOTING)),
                                                new ShootCommand(shooterSubsystem, intakeSubsystem)))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(shooterSubsystem::stopShooter),
                                                Commands.runOnce(() -> intakeSubsystem.runRoller(0)),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));
        }

        // --- LIFT (Elevator + Climber) ---
        private void configureLiftBindings() {
                // D-Pad Up: Lift Yukarı (manuel)
                operatorController.povUp()
                                .whileTrue(Commands.run(() -> liftSubsystem.setVoltage(6.0), liftSubsystem))
                                .onFalse(Commands.runOnce(liftSubsystem::stop, liftSubsystem));

                // D-Pad Down: Lift Aşağı (manuel)
                operatorController.povDown()
                                .whileTrue(Commands.run(() -> liftSubsystem.setVoltage(-4.0), liftSubsystem))
                                .onFalse(Commands.runOnce(liftSubsystem::stop, liftSubsystem));

                // A Button: Lift Level 1
                operatorController.a()
                                .onTrue(Commands.runOnce(() -> liftSubsystem.goToLevel(1), liftSubsystem));

                // B Button: Lift Level 2
                operatorController.b()
                                .onTrue(Commands.runOnce(() -> liftSubsystem.goToLevel(2), liftSubsystem));

                // Y Button: Lift Level 3
                operatorController.y()
                                .onTrue(Commands.runOnce(() -> liftSubsystem.goToLevel(3), liftSubsystem));

                // X Button: Lift Home (Level 0)
                operatorController.x()
                                .onTrue(Commands.runOnce(() -> liftSubsystem.goToLevel(0), liftSubsystem));

                // Left Bumper + Right Stick Up: Climb Extend
                operatorController.leftBumper()
                                .and(() -> operatorController.getRightY() < -0.5)
                                .whileTrue(Commands.parallel(
                                                Commands.runOnce(liftSubsystem::climbExtend, liftSubsystem),
                                                Commands.runOnce(() -> ledSubsystem
                                                                .setState(LEDSubsystem.LEDState.CLIMBING))))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(liftSubsystem::stop, liftSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));

                // Left Bumper + Right Stick Down: Climb Retract
                operatorController.leftBumper()
                                .and(() -> operatorController.getRightY() > 0.5)
                                .whileTrue(Commands.runOnce(liftSubsystem::climbRetract, liftSubsystem))
                                .onFalse(Commands.runOnce(liftSubsystem::stop, liftSubsystem));
        }

        // --- EMERGENCY ---
        private void configureEmergencyBindings() {
                // Start: Tüm mekanizmaları sıfırla (acil)
                operatorController.start()
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(liftSubsystem::stop),
                                                Commands.runOnce(feederSubsystem::stop),
                                                Commands.runOnce(shooterSubsystem::stopShooter),
                                                Commands.runOnce(() -> ledSubsystem
                                                                .setState(LEDSubsystem.LEDState.ERROR))));
        }
}
