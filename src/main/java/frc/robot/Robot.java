// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.feeder.FeederTestCommand;
import frc.robot.commands.intake.IntakeTestCommand;
import frc.robot.commands.shooter.ShooterTestCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

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

  // Track the last seen alliance to avoid redundant resets
  private Alliance m_lastAlliance = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // AdvantageKit Logger Initialization
    Logger.recordMetadata("ProjectName", "foxyCode2026");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Silence joystick connection warnings to reduce log spam and lag
    DriverStation.silenceJoystickConnectionWarning(true);

    Logger.start();

    // =====================================================================
    // SÜRÜCÜ KAMERASI USB PORT FORWARDING
    // =====================================================================
    // Limelight 3A sürücü kamerası RoboRIO'ya USB1 üzerinden bağlı.
    // USB bağlantısında Limelight varsayılan IP: 172.22.11.2
    // Port 5800 = MJPEG video akışı, Port 5801 = Web arayüzü
    // =====================================================================
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "172.29.0.1", port);
    }
    System.out.println("[PortForwarder] Sürücü kamerası (USB) → roborio.local:5800 (stream) / :5801 (web)");

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
    // Sürekli olarak FMS'den ittifak bilgisini kontrol et
    var currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent() && currentAlliance.get() != m_lastAlliance) {
      m_lastAlliance = currentAlliance.get();
      if (m_robotContainer != null) {
        m_robotContainer.resetToAllianceStart();
        System.out
            .println("[Disabled] FMS Alliance updated to " + m_lastAlliance.name() + ". Odometry and heading reset.");
      }
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    var alliance = DriverStation.getAlliance();
    
    System.out.println("[AutonomousInit] Alliance: " + (alliance.isPresent() ? alliance.get().name() : "None"));
    if (m_autonomousCommand != null) {
      System.out.println("[AutonomousInit] Starting Command: " + m_autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    } else {
      System.out.println("[AutonomousInit] WARNING: No autonomous command selected (null)");
    }

    // SMART POSE INIT
    if (m_robotContainer != null) {
      m_robotContainer.onAutonomousInit();
    }

    // Switch to main driver tab in Autonomous
    Shuffleboard.selectTab("SmartDashboard");
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
    Shuffleboard.selectTab("SmartDashboard");
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
    Shuffleboard.selectTab("Shooter Tuning");

    // Reset Pose for convenience
    resetPoseToAllianceStart();

    // Flywheel PID Tuning komutu başlat
    CommandScheduler.getInstance().schedule(
        new ShooterTestCommand(
            m_robotContainer.getShooterSubsystem(),
            m_robotContainer.getFeederSubsystem()));
    System.out.println("[Test Mode] ShooterTestCommand başlatıldı - Flywheel/Turret/Hood PID tuning aktif");

    // Intake Test komutu başlat
    CommandScheduler.getInstance().schedule(
        new IntakeTestCommand(
            m_robotContainer.getIntakeSubsystem()));
    System.out.println("[Test Mode] IntakeTestCommand başlatıldı - Pivot/Roller tuning aktif");

    // Feeder Test komutu başlat (ShooterTestCommand feeder'ı require ediyor, bu
    // yüzden ayrı çalışır)
    CommandScheduler.getInstance().schedule(
        new FeederTestCommand(
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
    var alliance = DriverStation.getAlliance();
    Pose2d startPose;

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      startPose = FieldConstants.kRedStartPose;
    } else {
      startPose = FieldConstants.kBlueStartPose;
    }

    // DriveSubsystem'i container üzerinden al ve resetle
    if (m_robotContainer != null) {
      DriveSubsystem drive = m_robotContainer.getDriveSubsystem();
      if (drive != null) {
        drive.resetOdometry(startPose);
        System.out.println("[Simulation] Reset Pose to "
            + alliance.orElse(Alliance.Blue) + " Start: " + startPose);
      }
    }
  }
}
