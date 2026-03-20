// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;



  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // AdvantageKit Logger Initialization
    Logger.recordMetadata("ProjectName", "foxyCode2026");

    if (isReal()) {
      Logger.addDataReceiver(new org.littletonrobotics.junction.wpilog.WPILOGWriter());
      Logger.addDataReceiver(new org.littletonrobotics.junction.networktables.NT4Publisher());
    } else {
      Logger.addDataReceiver(new org.littletonrobotics.junction.networktables.NT4Publisher());
    }

    // Silence joystick connection warnings to reduce log spam and lag
    edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);

    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // CRITICAL: Stop all shooter motors when disabled
    // This prevents the shooter from continuing after disable/restart
    if (m_robotContainer != null && m_robotContainer.getShooterSubsystem() != null) {
      m_robotContainer.getShooterSubsystem().stopAll();
      m_robotContainer.getShooterSubsystem().disableAutoAim();
      System.out.println("[Disabled] Shooter stopped and auto-aim disabled");
    }

    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // SMART POSE INIT
    if (m_robotContainer != null) {
      m_robotContainer.onAutonomousInit();
    }

    // Switch to main driver tab in Autonomous
    edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.selectTab("SmartDashboard");

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // SMART POSE INIT
    if (m_robotContainer != null) {
      m_robotContainer.onTeleopInit();
    }

    // Switch to main driver tab in Teleop
    edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.selectTab("SmartDashboard");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Switch to Shooter Tuning tab for live tuning
    edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.selectTab("Shooter Tuning");

    // Reset Pose for convenience
    resetPoseToAllianceStart();

    // Flywheel PID Tuning komutu başlat
    CommandScheduler.getInstance().schedule(
        new frc.robot.commands.shooter.ShooterTestCommand(
            m_robotContainer.getShooterSubsystem(),
            m_robotContainer.getFeederSubsystem()));
    System.out.println("[Test Mode] ShooterTestCommand başlatıldı - Flywheel/Turret/Hood PID tuning aktif");

    // Intake Test komutu başlat
    CommandScheduler.getInstance().schedule(
        new frc.robot.commands.intake.IntakeTestCommand(
            m_robotContainer.getIntakeSubsystem()));
    System.out.println("[Test Mode] IntakeTestCommand başlatıldı - Pivot/Roller tuning aktif");

    // Feeder Test komutu başlat (ShooterTestCommand feeder'ı require ediyor, bu
    // yüzden ayrı çalışır)
    CommandScheduler.getInstance().schedule(
        new frc.robot.commands.feeder.FeederTestCommand(
            m_robotContainer.getFeederSubsystem()));
    System.out.println("[Test Mode] FeederTestCommand başlatıldı - RPM tuning aktif");

    System.out.println("[Test Mode] Live tuning ENABLED - Dashboard değerleri okunuyor");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // CRITICAL: Run CommandScheduler so joystick triggers work in Test Mode
    // This is needed because joystick-based shooting tests require command
    // execution
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    resetPoseToAllianceStart();
  }

  /** This function is called periodically whilst in sim. */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Helper method to reset odometry to the correct alliance starting pose.
   * Useful for Simulation and Test modes.
   */
  private void resetPoseToAllianceStart() {
    // Get Alliance
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    edu.wpi.first.math.geometry.Pose2d startPose;

    if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
      startPose = frc.robot.constants.FieldConstants.kRedStartPose;
    } else {
      startPose = frc.robot.constants.FieldConstants.kBlueStartPose;
    }

    // DriveSubsystem'i container üzerinden al ve resetle
    if (m_robotContainer != null) {
      frc.robot.subsystems.drive.DriveSubsystem drive = m_robotContainer.getDriveSubsystem();
      if (drive != null) {
        drive.resetOdometry(startPose);
        System.out.println("[Simulation] Reset Pose to "
            + alliance.orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) + " Start: " + startPose);
      }
      // getDriveSubsystem wrapper methodu container'da olmayabilir, public field veya
      // getter eklenmeli.
      // Mevcut kodda DriveSubsystem getter yok mu? Kontrol edelim.
      // RobotContainer'a bakmadık ama getShooterSubsystem var.
      // Güvenli erişim için RobotContainer'a getter eklemek gerekebilir.
      // Şimdilik varsayım: public access veya getter var.
      // Kontrol: RobotContainer.java'yı okuduk mu? Evet.
      // RobotContainer koduna tekrar bakıp getter var mı emin olalım.
      // getShooterSubsystem var. getDriveSubsystem yoksa ekleyelim.
      // Şimdilik bu metodu burada bırakıp RobotContainer düzenlemesi yapacağım.
    }
  }
}
