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
import frc.robot.constants.FieldConstants; // Added Import

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Preferences;

public class DriveSubsystem extends SubsystemBase {

    // --- IO KATMANLARI (Inputs) ---
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final ModuleIO[] moduleIOs = new ModuleIO[4]; // 4 Modül (FL, FR, BL, BR)
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()
    };

    // --- SAFETY LAYER FIELDS ---
    private boolean safeDriveEnabled = true;
    private double fieldLength = 16.54; // Fallback (Crescendo default)
    private double fieldWidth = 8.21; // Fallback
    private double kRobotRadius; // Constructor'da hesaplanacak

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

        // Başlangıçta veri okumayı dene ki ilk pozisyon 0,0 olmasın (Encoderlar
        // absolute ise)
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);

        // Odometry Başlangıç (0,0 noktası ve 0 derece açısı ile başlar)
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.fromRadians(gyroInputs.yawPositionRad),
                getModulePositions(),
                new Pose2d());

        // --- SAFETY LAYER INIT ---
        // Robot Yarıçapı Hesapla (TrackWidth ve WheelBase'in hipotenüsünün yarısı +
        // tampon payı)
        kRobotRadius = Math.hypot(DriveConstants.kTrackWidthMeters, DriveConstants.kWheelBaseMeters) / 2.0 + 0.1;

        // Saha Düzeni Yükle (2026/2024)
        try {
            // AprilTagFields.k2024Crescendo veya kDefaultField kullanıyoruz (2026 sim
            // destekli)
            AprilTagFieldLayout layout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            fieldLength = layout.getFieldLength();
            fieldWidth = layout.getFieldWidth();
            Logger.recordOutput("Drive/Safety/FieldLength", fieldLength);
            Logger.recordOutput("Drive/Safety/FieldWidth", fieldWidth);
        } catch (Exception e) {
            DriverStation.reportError("Field Layout Yüklenemedi! Varsayılan değerler kullanılıyor.", false);
        }

        // Dashboard Toggle Oluştur (Varsayılan: True)
        Preferences.initBoolean("SafeDriveEnabled", true);

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Field", field);

        // --- PATHPLANNER AUTO BUILDER (2025 API) ---
        try {
            // RobotConfig dosyadan yüklenir (pathplanner/settings/robot.json)
            // PathPlanner AutoBuilder yapılandırması
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose, // Pose supplier
                    this::resetOdometry, // Pose reset
                    this::getChassisSpeeds, // Robot-relative speeds supplier
                    (speeds, feedforwards) -> runVelocity(speeds), // Robot-relative output (feedforward ignore)
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID (agresif sim için)
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (agresif sim için)
                    ),
                    config,
                    // Alliance'a göre yolu çevir (Red ise mirror)
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this);
        } catch (Exception e) {
            // RobotConfig yüklenemezse fallback oluştur
            DriverStation.reportError(
                    "PathPlanner RobotConfig dosyadan yüklenemedi, manuel config oluşturuluyor: " + e.getMessage(),
                    false);

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
                                1),
                        // Modül offsetleri (0.57m x 0.57m kare şase için)
                        // Center to module = 0.57 / 2 = 0.285
                        new Translation2d(0.285, 0.285),
                        new Translation2d(0.285, -0.285),
                        new Translation2d(-0.285, 0.285),
                        new Translation2d(-0.285, -0.285));

                AutoBuilder.configure(
                        this::getPose,
                        this::resetOdometry,
                        this::getChassisSpeeds,
                        (speeds, feedforwards) -> runVelocity(speeds),
                        new PPHolonomicDriveController(
                                // PID Katsayıları: P, I, D
                                // 1.0 az geldi, 3.0'a çıkarıldı.
                                new PIDConstants(3.0, 0.0, 0.0), // Translation PID
                                new PIDConstants(3.0, 0.0, 0.0) // Rotation PID
                        ),
                        manualConfig,
                        () -> {
                            var alliance = DriverStation.getAlliance();
                            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                        },
                        this);
            } catch (Exception ex) {
                DriverStation.reportError("Manuel PathPlanner Config oluşturulamadı: " + ex.getMessage(),
                        ex.getStackTrace());
            }
        }
    }

    // Field Visualization
    private final edu.wpi.first.wpilibj.smartdashboard.Field2d field = new edu.wpi.first.wpilibj.smartdashboard.Field2d();

    /**
     * Hedef pozisyonu sahada (Glass/SmartDashboard) gösterir.
     * 
     * @param target Hedef Pose
     */
    public void showTargetPose(Pose2d target) {
        field.getObject("Target").setPose(target);
    }

    /**
     * Hedef pozisyon göstergesini temizler.
     */
    public void clearTargetPose() {
        field.getObject("Target").setPoses(); // Boş liste ile temizle
    }

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
        // Eğer Gyro bağlı değilse veya koptuysa, Odometry'yi sadece encoderlarla
        // sürdürebiliriz
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
     * 
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

    /**
     * Teleop sürüş metodu.
     * 
     * @param xSpeed        İleri/Geri hız (m/s)
     * @param ySpeed        Sağ/Sol hız (m/s)
     * @param rot           Dönüş hızı (rad/s)
     * @param fieldRelative Saha merkezli mi?
     * @param rateLimit     Hızlanma limiti uygulansın mı?
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        // --- SAFETY LAYER (SMART CLAMPING V2) ---
        safeDriveEnabled = Preferences.getBoolean("SafeDriveEnabled", true);
        Logger.recordOutput("Drive/Safety/Enabled", safeDriveEnabled);

        // 1. Önce Hızları Analiz İçin Saha Merkezli (Field-Relative) Hale Getir
        // Eğer sürücü zaten field relative sürüyorsa xSpeed sahanın X'i demektir.
        // Eğer robot relative sürüyorsa, xSpeed robotun önüdür, sahada nereye denk
        // geldiğini bulmalıyız.
        ChassisSpeeds targetSpeeds;
        if (fieldRelative) {
            targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        } else {
            targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot); // Geçici, aşağıda çevireceğiz
            // Robot Relative hızları Saha Relative'e çevir ki sınırları kontrol edebilelim
            Rotation2d robotHeading = getPose().getRotation();
            // xSpeed (Forward), ySpeed (Left) -> Rotate by Heading -> Field X, Field Y
            double c = robotHeading.getCos();
            double s = robotHeading.getSin();
            double fieldVx = xSpeed * c - ySpeed * s;
            double fieldVy = xSpeed * s + ySpeed * c;
            targetSpeeds = new ChassisSpeeds(fieldVx, fieldVy, rot);
        }

        // Güvenlik açık ise müdahale et
        if (safeDriveEnabled) {
            Pose2d currentPose = getPose();
            double fieldVx = targetSpeeds.vxMetersPerSecond;
            double fieldVy = targetSpeeds.vyMetersPerSecond;

            // ---------------------------------------------------------
            // A. DİNAMİK SAHA SINIRLARI (Field Boundaries)
            // ---------------------------------------------------------
            // X Ekseni Kontrolü
            // Sol sınır (0)
            if (currentPose.getX() < kRobotRadius && fieldVx < 0) {
                fieldVx = 0;
            }
            // Sağ sınır (Length)
            if (currentPose.getX() > (fieldLength - kRobotRadius) && fieldVx > 0) {
                fieldVx = 0;
            }

            // Y Ekseni Kontrolü
            // Alt sınır (0)
            if (currentPose.getY() < kRobotRadius && fieldVy < 0) {
                fieldVy = 0;
            }
            // Üst sınır (Width)
            if (currentPose.getY() > (fieldWidth - kRobotRadius) && fieldVy > 0) {
                fieldVy = 0;
            }

            // ---------------------------------------------------------
            // B. YASAK BÖLGELER (Keep-Out Zones)
            // ---------------------------------------------------------
            for (Translation2d zoneCenter : FieldConstants.kKeepOutZones) {
                double dist = currentPose.getTranslation().getDistance(zoneCenter);
                // Güvenli yarıçap: Engel fiziksel boyutu (tahmini 0.5m) + Robot Yarıçapı
                double safeZoneRadius = 0.5 + kRobotRadius;

                if (dist < safeZoneRadius) {
                    // Robot bölgeye çok yakın veya içinde!

                    // Engelden robota doğru olan vektör (Kaçış vektörü)
                    Translation2d vecRobotToZone = zoneCenter.minus(currentPose.getTranslation());

                    // Hız vektörümüz engele doğru mu? (Dot Product > 0 ise engele gidiyoruz)
                    // Hız vektörü: (fieldVx, fieldVy)
                    // Zone vektörü: (dx, dy)
                    double dotProduct = (fieldVx * vecRobotToZone.getX()) + (fieldVy * vecRobotToZone.getY());

                    if (dotProduct > 0) {
                        // Engele doğru gidiyoruz! Hız vektörünü durdur veya engeli teğet geçecek
                        // şekilde yansıt.
                        // Basitlik için: Engele doğru olan hareketi tamamen kesiyoruz.
                        fieldVx = 0;
                        fieldVy = 0;
                        Logger.recordOutput("Drive/Safety/ObstacleIntervention", true);
                    }
                }
            }

            // Güvenli hızları geri ata
            targetSpeeds = new ChassisSpeeds(fieldVx, fieldVy, rot);

            // ---------------------------------------------------------
            // C. KOORDİNAT SİSTEMİNİ GERİ ÇEVİR (Gerekirse)
            // ---------------------------------------------------------
            // Eğer kullanıcı robot-relative istemişse, ama biz field-relative üzerinde
            // düzenleme yaptık.
            // Şimdi bu 'güvenli' field hızlarını tekrar robot'un o anki açısına göre
            // robot-relative'e çevirmeliyiz
            // ki runVelocity veya sonraki adımlar doğru çalışsın.
            // VEYA: runVelocity zaten Robot Relative istiyorsa, her halükarda Robot
            // Relative dönmeliyiz.
            // runVelocity metodu: setpointStates = kinematics.toSwerveModuleStates(speeds)
            // kullanıyor.
            // kinematics.toSwerveModuleStates her zaman ROBOT RELATIVE hız bekler!

            // Dolayısıyla, elimizdeki targetSpeeds (Field Relative) şu an.
            // Bunu Robot Relative'e çevirmemiz ŞART.
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getPose().getRotation());
        } else {
            // Güvenlik kapalıysa ve Field Relative isteniyorsa, Robot Relative'e çevirip
            // gönderelim
            // (runVelocity robot relative bekler)
            if (fieldRelative) {
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getPose().getRotation());
            }
            // fieldRelative değilse zaten targetSpeeds (ilk atamada) robot relative idi.
        }

        runVelocity(targetSpeeds);
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
                    Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad));

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
     * 
     * @param visionPose Kameradan gelen tahmini pozisyon
     * @param timestamp  Görüntünün alındığı zaman
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        // Saha dışı kontrolü
        if (visionPose.getX() < 0 || visionPose.getX() > 17 ||
                visionPose.getY() < 0 || visionPose.getY() > 9) {
            return; // Saçma veri, ekleme
        }

        // Standart sapma ile ekle (titreme önleme)
        // Değerler: x (metre), y (metre), rotation (radyan)
        // Daha yüksek değer = daha az güven = daha az titreme
        poseEstimator.addVisionMeasurement(
                visionPose,
                timestamp,
                edu.wpi.first.math.VecBuilder.fill(0.5, 0.5, 0.5) // Standart sapma değerleri
        );
    }

    /**
     * Vision Subsystem'den gelen veriyi mesafe bazlı güvenle Odometry'ye ekler.
     * Yakın tag = yüksek güven, uzak tag = düşük güven.
     * 
     * @param visionPose     Kameradan gelen tahmini pozisyon
     * @param timestamp      Görüntünün alındığı zaman
     * @param avgTagDistance Tag'lere ortalama mesafe (metre)
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, double avgTagDistance) {
        // Saha dışı kontrolü
        if (visionPose.getX() < 0 || visionPose.getX() > 17 ||
                visionPose.getY() < 0 || visionPose.getY() > 9) {
            return; // Saçma veri, ekleme
        }

        // Mesafeye göre standart sapma hesapla
        // Yakın (0.5m) = 0.1 std dev (yüksek güven)
        // Uzak (4m) = 0.9 std dev (düşük güven)
        double baseStdDev = 0.1;
        double distanceMultiplier = avgTagDistance * 0.2; // Her metre için 0.2 ekle
        double stdDev = Math.min(baseStdDev + distanceMultiplier, 1.0); // Max 1.0

        poseEstimator.addVisionMeasurement(
                visionPose,
                timestamp,
                edu.wpi.first.math.VecBuilder.fill(stdDev, stdDev, stdDev * 2) // Açı için 2x
        );
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
        // Odometry'yi sıfırlarken mevcut pozisyonu koruyup sadece açıyı sıfırlamak
        // istersek:
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
                pose);
    }

    /**
     * Modül Pozisyonlarını (Metre ve Açı) Okur
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(
                    // Dişli oranı hesaba katılmalı: Motor Rad / Reduction * Radius = Metre
                    (moduleInputs[i].drivePositionRad / ModuleConstants.kDrivingMotorReduction)
                            * (ModuleConstants.kWheelDiameterMeters / 2.0),
                    Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad));
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
                    (moduleInputs[i].driveVelocityRadPerSec / ModuleConstants.kDrivingMotorReduction)
                            * (ModuleConstants.kWheelDiameterMeters / 2.0),
                    Rotation2d.fromRadians(moduleInputs[i].turnAbsolutePositionRad));
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

    public double getPitch() {
        return gyroInputs.pitchDegrees;
    }

    public double getRoll() {
        return gyroInputs.rollDegrees;
    }

    /**
     * Gyro açısal hızını döndürür (rad/s).
     * Vision MegaTag 2 senkronizasyonu için gereklidir.
     */
    public double getGyroVelocityRadPerSec() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    /**
     * Vision Subsystem'den gelen veriyi dinamik standart sapma (Trust Matrix) ile
     * ekler.
     * 
     * @param visionPose Kameradan gelen tahmini pozisyon
     * @param timestamp  Görüntünün alındığı zaman (Latency hesaplanmış)
     * @param stdDevs    Güvenilirlik matrisi (x, y, theta)
     */
    public void addVisionMeasurement(edu.wpi.first.math.geometry.Pose2d visionPose, double timestamp,
            edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    public edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        return getPose().getRotation();
    }
}