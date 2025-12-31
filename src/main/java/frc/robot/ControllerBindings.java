package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;

import frc.robot.commands.drive.XStanceCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.grabber.GrabCommand;
import frc.robot.commands.grabber.ReleaseCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.climber.ClimbDownCommand;

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
 * <li><b>Elevator:</b> D-Pad Yukarı/Aşağı</li>
 * <li><b>Arm:</b> D-Pad Sol/Sağ</li>
 * <li><b>Grabber:</b> A/B butonları</li>
 * <li><b>Wrist:</b> X/Y butonları</li>
 * <li><b>Climber:</b> Sol Bumper + Sağ Stick</li>
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
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private final GrabberSubsystem grabberSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final WristSubsystem wristSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final LEDSubsystem ledSubsystem;

    public ControllerBindings(
            CommandXboxController driverController,
            CommandXboxController operatorController,
            DriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem,
            GrabberSubsystem grabberSubsystem,
            ClimberSubsystem climberSubsystem,
            WristSubsystem wristSubsystem,
            FeederSubsystem feederSubsystem,
            LEDSubsystem ledSubsystem) {

        this.driverController = driverController;
        this.operatorController = operatorController;
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.wristSubsystem = wristSubsystem;
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
        // SIM MODE: A ve B ile dönüş (simülasyon testi için)
        // A: Sola Dön
        driverController.a()
                .whileTrue(Commands.run(() -> driveSubsystem.runVelocity(
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 3.0)), driveSubsystem));

        // B: Sağa Dön
        driverController.b()
                .whileTrue(Commands.run(() -> driveSubsystem.runVelocity(
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, -3.0)), driveSubsystem));

        // X: Fren (X-Stance)
        driverController.x()
                .whileTrue(new XStanceCommand(driveSubsystem));

        // Y: RobotContainer'da PathPlanner ile kullanılıyor (Speaker'a Git)

        // Left Bumper: Yavaş Sola Dön (kademeli)
        driverController.leftBumper()
                .whileTrue(Commands.run(() -> driveSubsystem.runVelocity(
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 1.5)), driveSubsystem));

        // Right Bumper: Yavaş Sağa Dön (kademeli)
        driverController.rightBumper()
                .whileTrue(Commands.run(() -> driveSubsystem.runVelocity(
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, -1.5)), driveSubsystem));
    }

    // ==================== OPERATOR BINDINGS ====================
    private void configureOperatorBindings() {
        configureIntakeBindings();
        configureShooterBindings();
        configureElevatorBindings();
        configureArmBindings();
        configureGrabberBindings();
        configureWristBindings();
        configureClimberBindings();
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
                        Commands.runOnce(() -> ledSubsystem.setState(LEDSubsystem.LEDState.SHOOTING)),
                        new ShootCommand(shooterSubsystem, intakeSubsystem)))
                .onFalse(Commands.parallel(
                        Commands.runOnce(shooterSubsystem::stopShooter),
                        Commands.runOnce(() -> intakeSubsystem.runRoller(0)),
                        Commands.runOnce(() -> ledSubsystem.setIdle())));
    }

    // --- ELEVATOR ---
    private void configureElevatorBindings() {
        // D-Pad Up: Elevator Yukarı
        operatorController.povUp()
                .whileTrue(Commands.run(() -> elevatorSubsystem.setVoltage(6.0), elevatorSubsystem))
                .onFalse(Commands.runOnce(elevatorSubsystem::stop, elevatorSubsystem));

        // D-Pad Down: Elevator Aşağı
        operatorController.povDown()
                .whileTrue(Commands.run(() -> elevatorSubsystem.setVoltage(-4.0), elevatorSubsystem))
                .onFalse(Commands.runOnce(elevatorSubsystem::stop, elevatorSubsystem));
    }

    // --- ARM ---
    private void configureArmBindings() {
        // D-Pad Right: Arm Yukarı
        operatorController.povRight()
                .whileTrue(Commands.run(() -> armSubsystem.setVoltage(3.0), armSubsystem))
                .onFalse(Commands.runOnce(armSubsystem::stop, armSubsystem));

        // D-Pad Left: Arm Aşağı
        operatorController.povLeft()
                .whileTrue(Commands.run(() -> armSubsystem.setVoltage(-2.0), armSubsystem))
                .onFalse(Commands.runOnce(armSubsystem::stop, armSubsystem));
    }

    // --- GRABBER ---
    private void configureGrabberBindings() {
        // A: Grabber Kapat (Tut)
        operatorController.a()
                .onTrue(new GrabCommand(grabberSubsystem));

        // B: Grabber Aç (Bırak)
        operatorController.b()
                .onTrue(new ReleaseCommand(grabberSubsystem));
    }

    // --- WRIST ---
    private void configureWristBindings() {
        // X: Wrist Center
        operatorController.x()
                .onTrue(Commands.runOnce(() -> wristSubsystem.goToCenter(), wristSubsystem));

        // Y: Wrist Score
        operatorController.y()
                .onTrue(Commands.runOnce(() -> wristSubsystem.goToScore(), wristSubsystem));
    }

    // --- CLIMBER ---
    private void configureClimberBindings() {
        // Left Bumper + Right Stick Up: Climb Up
        operatorController.leftBumper()
                .and(() -> operatorController.getRightY() < -0.5)
                .whileTrue(Commands.parallel(
                        new ClimbUpCommand(climberSubsystem),
                        Commands.runOnce(() -> ledSubsystem.setState(LEDSubsystem.LEDState.CLIMBING))))
                .onFalse(Commands.parallel(
                        Commands.runOnce(climberSubsystem::stop, climberSubsystem),
                        Commands.runOnce(() -> ledSubsystem.setIdle())));

        // Left Bumper + Right Stick Down: Climb Down
        operatorController.leftBumper()
                .and(() -> operatorController.getRightY() > 0.5)
                .whileTrue(new ClimbDownCommand(climberSubsystem))
                .onFalse(Commands.runOnce(climberSubsystem::stop, climberSubsystem));
    }

    // --- EMERGENCY ---
    private void configureEmergencyBindings() {
        // Start: Tüm mekanizmaları sıfırla (acil)
        operatorController.start()
                .onTrue(Commands.parallel(
                        Commands.runOnce(elevatorSubsystem::stop),
                        Commands.runOnce(armSubsystem::stop),
                        Commands.runOnce(grabberSubsystem::stop),
                        Commands.runOnce(climberSubsystem::stop),
                        Commands.runOnce(wristSubsystem::stop),
                        Commands.runOnce(feederSubsystem::stop),
                        Commands.runOnce(shooterSubsystem::stopShooter),
                        Commands.runOnce(() -> ledSubsystem.setState(LEDSubsystem.LEDState.ERROR))));
    }
}
