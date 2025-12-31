package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {

    // --- IO KATMANLARI (Inputs) ---
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final ModuleIO[] moduleIOs = new ModuleIO[4]; // 4 Modül (FL, FR, BL, BR)
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
        new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()
    };

    // --- ODOMETRY & KINEMATICS ---
    private final SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * Constructor
     */
    public DriveSubsystem(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        this.gyroIO = gyroIO;
        this.moduleIOs[0] = fl;
        this.moduleIOs[1] = fr;
        this.moduleIOs[2] = bl;
        this.moduleIOs[3] = br;

        // Başlangıçta veri okumayı dene ki ilk pozisyon 0,0 olmasın (Encoderlar absolute ise)
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);

        // Odometry Başlangıç (0,0 noktası ve 0 derece açısı ile başlar)
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            Rotation2d.fromRadians(gyroInputs.yawPositionRad), 
            getModulePositions(), 
            new Pose2d()
        );

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Field", field);

        // --- PATHPLANNER AUTO BUILDER (2025 API) ---
        try {
            // RobotConfig dosyadan yüklenir (pathplanner/settings/robot.json)
            // PathPlanner AutoBuilder yapılandırması
            RobotConfig config = RobotConfig.fromGUISettings();
            
            AutoBuilder.configure(
                this::getPose,                // Pose supplier
                this::resetOdometry,          // Pose reset
                this::getChassisSpeeds,       // Robot-relative speeds supplier
                (speeds, feedforwards) -> runVelocity(speeds),  // Robot-relative output (feedforward ignore)
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),   // Translation PID (agresif sim için)
                    new PIDConstants(5.0, 0.0, 0.0)   // Rotation PID (agresif sim için)
                ),
                config,
                // Alliance'a göre yolu çevir (Red ise mirror)
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            // RobotConfig yüklenemezse fallback oluştur
            DriverStation.reportError("PathPlanner RobotConfig dosyadan yüklenemedi, manuel config oluşturuluyor: " + e.getMessage(), false);
            
            try {
                 // Manuel Config (Robota özel gerçek veriler)
                 // NOT: PathPlanner GUI'sında "0 Acceleration" hatası olduğundan,
                 // motor özelliklerini burada manuel olarak en doğru şekilde giriyoruz.
                 // NEO Vortex Gerçek Özellikleri:
                 // Stall Torque: 3.6 Nm, Stall Current: 211 A, Free Speed: 6784 RPM
                 DCMotor customMotor = new DCMotor(12.0, 3.6, 211.0, 3.6, 6784.0 / 60.0 * 2.0 * Math.PI, 1);

                 RobotConfig manualConfig = new RobotConfig(
                    50.0, // Mass kg (Kullanıcının verisi)
                    6.0, // MOI
                    new com.pathplanner.lib.config.ModuleConfig(
                        ModuleConstants.kWheelDiameterMeters / 2.0, // 3 inç yarıçap (0.0381m)
                        4.5, // Max Speed
                        1.2, // Wheel COF
                        customMotor,
                        60, // Current Limit (Kullanıcının verisi)
                        1
                    ),
                    // Modül offsetleri (0.57m x 0.57m kare şase için)
                    // Center to module = 0.57 / 2 = 0.285
                    new Translation2d(0.285, 0.285),
                    new Translation2d(0.285, -0.285),
                    new Translation2d(-0.285, 0.285),
                    new Translation2d(-0.285, -0.285)
                 );

                 AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getChassisSpeeds,
                    (speeds, feedforwards) -> runVelocity(speeds),
                    new PPHolonomicDriveController(
                        // PID Katsayıları: P, I, D
                        // 1.0 az geldi, 3.0'a çıkarıldı.
                        new PIDConstants(3.0, 0.0, 0.0), // Translation PID
                        new PIDConstants(3.0, 0.0, 0.0)  // Rotation PID
                    ),
                    manualConfig,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this
                );
            } catch (Exception ex) {
                DriverStation.reportError("Manuel PathPlanner Config oluşturulamadı: " + ex.getMessage(), ex.getStackTrace());
            }
        }
    }
    
    // Field Visualization
    private final edu.wpi.first.wpilibj.smartdashboard.Field2d field = new edu.wpi.first.wpilibj.smartdashboard.Field2d();

    @Override
    public void periodic() {
        // 1. VERİ OKUMA VE LOGLAMA (AdvantageKit)
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Drive/Module" + i, moduleInputs[i]);
        }

        // 2. ODOMETRY GÜNCELLEME (Robot Nerede?)
        // Eğer Gyro bağlı değilse veya koptuysa, Odometry'yi sadece encoderlarla sürdürebiliriz 
        // ama Swerve için Gyro şarttır.
        Rotation2d gyroAngle = Rotation2d.fromRadians(gyroInputs.yawPositionRad);
        
        poseEstimator.update(gyroAngle, getModulePositions());
        field.setRobotPose(getPose());

        // 3. LOGLAMA (Görselleştirme)
        // Robotun tahmini konumunu logla (AdvantageScope 3D sahasında görünür)
        Logger.recordOutput("Odometry/Robot", getPose());
        
        // Modüllerin gerçek durumunu logla
        Logger.recordOutput("SwerveStates/Real", getModuleStates());

        // --- SIMULATION PHYSICS UPDATE ---
        if (frc.robot.constants.Constants.currentMode == frc.robot.constants.Constants.Mode.SIM) {
            // Modüllerin o anki hız ve açılarından robotun hareketini hesapla
            var moduleStates = getModuleStates();
            var chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);

            // Gyro Simülasyonunu Güncelle (Robotun döndüğünü Gyro bilmeli)
            // Bu sayede Odometry açısı güncellenir ve robot döner.
            if (gyroIO instanceof GyroIOSim) {
                ((GyroIOSim) gyroIO).setYawVelocity(chassisSpeeds.omegaRadiansPerSecond);
            }
        }
    }

    /**
     * Robotu Sür (Teleop veya Otonom)
     * @param speeds İstenen X hızı, Y hızı ve Dönüş hızı
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // DEBUG: PathPlanner'dan gelen hızları logla
        Logger.recordOutput("Drive/Debug/CommandedSpeeds/Vx", speeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/Debug/CommandedSpeeds/Vy", speeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/Debug/CommandedSpeeds/Omega", speeds.omegaRadiansPerSecond);
        
        // Şase hızlarını modül durumlarına (State) çevir
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

        // Modüllere emir gönder
        setModuleStates(setpointStates);
    }

    public void stop() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
        setModuleStates(states);
    }

    private void setModuleStates(SwerveModuleState[] states) {
        // Hedef durumu logla
        Logger.recordOutput("Drive/Debug/SwerveStates/Setpoint", states);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState optimizedState = SwerveModuleState.optimize(
                states[i], 
                Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad)
            );
            
            // 1. Dönüş (Turn) Kontrolü
            moduleIOs[i].setTurnPosition(optimizedState.angle.getRadians());

            // 2. Sürüş (Drive) Kontrolü - FeedForward (Open Loop)
            double volts = 0.0;
            // Daha hassas kontrol için deadband daha da düşürüldü
            if (Math.abs(optimizedState.speedMetersPerSecond) > 0.0001) {
                 volts = (optimizedState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond) * 12.0;
            }
            
            moduleIOs[i].setDriveVoltage(volts);
            
            // DEBUG: Her modül için hesaplanan voltajı logla
            Logger.recordOutput("Drive/Debug/Module" + i + "/TargetSpeed", optimizedState.speedMetersPerSecond);
            Logger.recordOutput("Drive/Debug/Module" + i + "/AppliedVolts", volts);
        }
    }

    /**
     * Vision Subsystem'den gelen veriyi Odometry'ye ekler.
     * Bu metod sayesinde robot zamanla kayan konumunu kamera ile düzeltir.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        // Güvenlik: Eğer kamera saçma bir yerde (örn: saha dışı) robot görüyorsa ekleme.
        // Şimdilik direkt ekliyoruz.
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /**
     * Anlık Robot Konumunu Döndürür (Field Relative)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gyro'yu sıfırlar (Saha Merkezli Sürüşü Resetler)
     */
    public void zeroHeading() {
        gyroIO.zeroHeading();
        // Odometry'yi sıfırlarken mevcut pozisyonu koruyup sadece açıyı sıfırlamak istersek:
        Pose2d currentPose = getPose();
        Pose2d newPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());
        resetOdometry(newPose);
    }

    /**
     * Odometry'yi belirli bir konuma ışınlar (Otonom başlangıcı için)
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
            Rotation2d.fromRadians(gyroInputs.yawPositionRad), 
            getModulePositions(), 
            pose
        );
    }

    /**
     * Modül Pozisyonlarını (Metre ve Açı) Okur
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(
                // Dişli oranı hesaba katılmalı: Motor Rad / Reduction * Radius = Metre
                (moduleInputs[i].drivePositionRad / ModuleConstants.kDrivingMotorReduction) * (ModuleConstants.kWheelDiameterMeters / 2.0),
                Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad)
            );
        }
        return positions;
    }

    /**
     * Modül Durumlarını (Hız ve Açı) Okur
     */
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(
                // Dişli oranı hesaba katılmalı: Motor Rad/s / Reduction * Radius = m/s
                (moduleInputs[i].driveVelocityRadPerSec / ModuleConstants.kDrivingMotorReduction) * (ModuleConstants.kWheelDiameterMeters / 2.0),
                Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad)
            );
        }
        return states;
    }

    @Override
    public void simulationPeriodic() {
        // Gyro Simülasyonu: Kinematik modelden açısal hızı alıp Gyro'ya ver
        // Bu sayede robot simülasyonda kendi ekseni etrafında dönebilir.
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        gyroIO.setYawVelocity(speeds.omegaRadiansPerSecond);
    }

    /**
     * Robotun o anki hızını ChassisSpeeds olarak döndürür.
     * PathPlanner için gerekli.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
}