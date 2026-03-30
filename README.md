# FRC Team Laissez Faire — 2026 Sezon Robot Kodu

Bu depo, **LF 2026 Sezonu** robotunun WPILib (Java) tabanlı resmi yazılımını içerir. Gelişmiş otonom navigasyon, çift Limelight görüş füzyonu, hareket halinde atış (Shoot-on-the-Move), özelleştirilebilir saha koordinatları ve optimize edilmiş alt sistemler (Intake, Feeder, Shooter, Swerve Drive) barındırır.

---

## 🚀 Temel Özellikler

### 🎮 Tek Sürücü Kurulumu (Birleşik Kontrol)
 sürüş hareketleri hem de mekanizma operasyonları doğrudan tek bir **Sürücü Kontrolcüsü** (Port 0) üzerinden yönetilmektedir.

| Tuş | İşlev | Tetikleme |
|---|---|---|
| **Sağ Tetik** | Akıllı Atış — Pozisyona göre RPM, Hood, Taret otomatik hesaplanır | Basılı tutulduğu sürece |
| **Sol Tetik** | Intake Roller Aç/Kapa (Toggle) — İlk basışta çalışır, tekrar basınca durur | Tek basış |
| **Sağ Tampon** | Otomatik TrenchPass — İttifak ve konuma göre A↔B geçişi | Basılı tutulduğu sürece |
| **Sol Tampon** | Otomatik BumpPass — İttifak ve konuma göre güvenli Bump geçişi | Basılı tutulduğu sürece |
| **A Tuşu** | Intake Dışarı Çıkar (Deploy) | Tek basış |
| **B Tuşu** | Intake İçeri Çek (Retract) | Tek basış |
| **X Tuşu** | Kusma — Intake + Feeder ters yönde çalışır | Basılı tutulduğu sürece |
| **Y Tuşu** | Hood Açısını Sıfırla (0° kapanma) | Tek basış |
| **POV Yukarı/Aşağı** | Hub Hedef Y Ofseti (±0.1m) | Tek basış |
| **POV Sol/Sağ** | Hub Hedef X Ofseti (±0.1m) | Tek basış |

---

### 🎯 Atıcı Alt Sistemi (Shooter)
Atıcı sistemi, robotun sahadaki konumuna göre otomatik hedefleme yapar.

- **Taret**: NEO motor + Relative Encoder — REVLib Onboard PID ile ±180° soft limit, derece cinsinden pozisyon kontrolü
- **Hood**: NEO motor + Relative Encoder — REVLib Onboard PID, homing destekli, 0°–45° aralığı
- **Flywheel**: Kraken X60 — Phoenix6 Velocity Control, RPM tabanlı
- **Fizik Tabanlı Nişan Alma**: Taret RPM'leri ve Hood açıları, robotun `HubCenter` hedefine olan 2D mesafesine göre `InterpolatingDoubleTreeMap` ile otomatik hesaplanır
- **Hub Ofset Ayarı**: Yön tuşları (POV) ile hedef nokta ±0.1m hassasiyetle canlı kaydırılarak ince ayar yapılabilir
- **Shoot-on-the-Move**: `MovingShootUtil` sınıfı, robotun anlık hız vektörünü ve topun uçuş süresini (ToF) kullanarak sanal hedef hesaplar. Robot duruyorsa, klasik statik atış sistemiyle birebir aynı sonucu verir

### ⚙️ Besleyici Alt Sistemi (Feeder)
İkili motor mimarisi ile çalışır:

- **Indexer (SparkMax NEO)**: Topu atış bölgesine indeksler — Velocity PID
- **Kicker (SparkMax NEO)**: Topu fırlatma noktasına iter — Velocity PID
- Tüm PID ve hız (RPM) parametreleri Dashboard'dan ayarlanabilir ve RIO'ya kaydedilir
- Manuel override desteği (test/hata ayıklama için)

### 📏 Toplama Alt Sistemi (Intake)
2026 sezonunda pivot tasarımı terk edilmiş, yerini **Lineer Uzatma (Rack & Pinion)** mekanizması almıştır.

- **Extension Motoru**: SparkMax (NEO 1.2) — Pozisyon PID, Relative Encoder, Homing desteği
- **Roller Motoru**: Kraken X60 (TalonFX) — Velocity PID, RPM tabanlı
- Dişli oranı ve pinyon çapı Dashboard'dan ayarlanabilir (motor devri ↔ cm dönüşümü otomatik)
- Yazılımsal soft limit'ler (Retracted/Deployed cm cinsinden)
- Homing komutu: Düşük voltajda geriye sürer, encoder'ı sıfırlar

### 🏎️ Swerve Sürüş Alt Sistemi (Drive)
MAXSwerve modül yapısı (SparkFlex + SparkMax) üzerine kuruludur.

- **Pose Tahmini**: `SwerveDrivePoseEstimator` ile odometri + çift Limelight görüş füzyonu
- **Güvenlik Katmanı**: Saha sınırı (16.54m × 8.21m) ve Keep-Out bölgelerinde otomatik hız sıfırlama
- **Hız Çarpanları**: Teleop, Otonom ve Feed modu için ayrı ayrı Dashboard'dan ayarlanabilir
- **Dönüş Merkezi**: Atış sırasında taret merkezine dinamik kaydırma desteği (`setCenterOfRotation`)
- **Motor İnversiyon**: Her sürüş ve dönüş motoru Dashboard üzerinden canlı ters çevrilebilir (`TunableBoolean`)

### 👁️ Görüş Alt Sistemi (Vision)
Çift Limelight kamera ile MegaTag 2 pose füzyonu:

- **limelight-left**: Limelight 4, arka-sol köşe, 30° dışa bakış
- **limelight-right**: Limelight 3, arka-sağ köşe, 30° dışa bakış
- NavX gyro senkronizasyonu (MegaTag 2 gereksinimi)
- Dinamik güven matrisi: Tag sayısı, mesafe ve dönüş hızına göre standart sapma hesaplanır
- Saha dışı ve yüksek gyro hızı filtresi ile hatalı ölçüm reddi
- Dashboard üzerinden `Vision/Enabled` toggle ile açılıp kapatılabilir

### 💡 LED Alt Sistemi
Robot durumuna göre LED geri bildirim sağlar.

---

## 🛣️ Akıllı Geçiş Komutları

| Komut | Açıklama |
|---|---|
| `TrenchPassCommand` | İttifak ve robotun Y konumuna göre (üst/alt yarı) uygun Trench geçiş noktalarını seçer, en yakın noktaya önce gider |
| `BumpPassCommand` | Aynı mantıkla Bump bölgesinden güvenli geçiş sağlar |
| `SimpleDriveToPose` | Holonomic PID kontrolcü ile hedefe otonom sürüş |

Her iki geçiş komutu da çalışırken Hood açısını otomatik olarak 0°'a konumlar.

---

## 🤖 Otonom Senaryolar

`AutonomousScenarios.java` dosyasından `SendableChooser` ile seçilebilir:

| Senaryo | Açıklama |
|---|---|
| **0 — Hiçbir Şey Yapma** | Güvenli varsayılan |
| **1 — Topla ve At** | Geçiş → Manuel Noktaya Intake ile Sürüş → BumpPass ile Dönüş → Atış |
| **2 — Source'dan Al, At** | Source noktasına intake çalıştırarak git → Varınca AutoShoot |
| **3 — Çift Topla ve At** | İki tam toplama + atış döngüsü (1. ve 2. tur ayrı dönüş noktaları) |

### Dashboard Seçiciler
- **Pass Mode**: Trench veya Bump geçiş rotası
- **Start Side**: Sol veya Sağ başlangıç

### Manuel Dönüş Noktaları
Toplama sonrası dönüş noktaları, `getManualReturnPoint()` metodunda 8 farklı senaryo için ayrı ayrı tanımlanmıştır:

| İttifak | Taraf | 1. Tur | 2. Tur |
|---|---|---|---|
| Kırmızı | Sol | `(8.8, 2.6)` | `(8, 4)` |
| Kırmızı | Sağ | `(8.8, 5.6)` | `(8, 4)` |
| Mavi | Sol | `(7.8, 5.7)` | `(8, 4)` |
| Mavi | Sağ | `(7.8, 2.4)` | `(8, 4)` |

> Sol/Sağ belirleme: Robotun o anki Y konumuna göre otomatik hesaplanır.
> Kırmızı ittifakta Y > 4.0 ise Sağ, Mavi ittifakta Y < 4.0 ise Sağ kabul edilir.

Robot, bu noktalara giderken **önce yüzünü hedefe döner**, sonra doğrusal hareketle sürerken aynı anda Intake + Feeder çalıştırır.

---

## 🔧 Dashboard Ayar Sistemi

Tüm kritik parametreler RoboRIO flash belleğine WPILib `Preferences` API'si ile kaydedilir.

### TunableNumber ve TunableBoolean
- **`TunableNumber`**: Double değerler — Dashboard'dan düzenlenebilir, RIO'ya kaydedilir (`/Preferences/[grup]/[anahtar]`)
- **`TunableBoolean`**: Boolean değerler — Dashboard'da anahtar/toggle olarak görünür

### Hız Çarpanları

| Dashboard Yolu | Mod | Varsayılan |
|---|---|---|
| `Tuning/Drive/TeleopSpeedMultiplier` | Teleop | 1.0 |
| `Tuning/Auto/SpeedMultiplier` | Otonom | 1.0 |
| `Tuning/Auto/FeedSpeedMultiplier` | Otonom (Toplama) | 0.5 |

### Motor İnversiyon Kontrolleri (Canlı Toggle)
Swerve motor inversiyon ayarları `TunableBoolean` anahtarları ile yönetilir:
- `Drive/Inverts/Driving/` — FL, FR, RL, RR sürüş motorları (SparkFlex)
- `Drive/Inverts/Turning/` — FL, FR, RL, RR dönüş motorları (SparkMax)

Bu değerler:
1. Robot açılışında RIO'dan **yüklenir**
2. Motor kontrolcülerine **anında uygulanır**
3. Teleop ve Test modlarında Dashboard'dan **canlı değiştirilebilir**

### Saha Koordinatları
Tüm saha pozisyonları (Hub, Pass hedefleri, Trench/Bump geçiş noktaları, Source) `TunableNumber` olarak `/Preferences/Field/...` altında tanımlıdır ve yeniden derleme gerektirmeden ayarlanabilir.

---

## 💻 Kod Mimarisi

### Ana Sınıflar
| Dosya | Açıklama |
|---|---|
| `Robot.java` | Yaşam döngüsü yönetimi (teleopInit, autonomousInit, simülasyon) |
| `RobotContainer.java` | Alt sistem oluşturma ve komut atama |
| `ControllerBindings.java` | Tek sürücü kontrolcü tuş atamaları |
| `AutonomousScenarios.java` | Otonom rutinleri — `SendableChooser` ve `DeferredCommand` ile çalışma anında çözümleme |
| `Configs.java` | Motor ve mekanizma yapılandırma sabitleri |

### Alt Sistemler (`subsystems/`)
| Alt Sistem | Açıklama |
|---|---|
| `DriveSubsystem` | MAXSwerve sürüş, PoseEstimator, güvenlik katmanı, hız çarpanları |
| `ShooterSubsystem` | Taret + Hood + Flywheel, otomatik nişan, hareketli atış desteği |
| `FeederSubsystem` | İkili motor (Indexer + Kicker), Velocity PID |
| `IntakeSubsystem` | Lineer uzatma (Rack & Pinion) + Kraken roller |
| `VisionSubsystem` | Çift Limelight MegaTag 2 pose füzyonu |
| `LEDSubsystem` | Robot durumu LED geri bildirimi |

### Komutlar (`commands/`)
| Komut | Açıklama |
|---|---|
| `ShootCommand` | Teleop akıllı atış — pozisyona göre RPM/Hood/Taret hesaplar, hazır olunca ateşler |
| `AutoShootCommand` | Otonom atış — zaman sınırlı, ateş kilidi mekanizmalı |
| `FixedShotCommand` | Sabit parametreli atış (test/kalibrasyon) |
| `SimpleDriveToPose` | Holonomic PID ile hedefe sürüş |
| `TrenchPassCommand` | İttifak-konuma duyarlı Trench geçişi |
| `BumpPassCommand` | İttifak-konuma duyarlı Bump geçişi |
| `FeedPassCommand` | Toplama rotası boyunca sürerken intake çalıştırma |
| `DriveWithJoystick` | Teleop joystick sürüş, ivme sınırlayıcı |

### Sabitler (`constants/`)
| Dosya | Açıklama |
|---|---|
| `FieldConstants` | Tüm saha koordinatları (TunableNumber), AprilTag layout, keep-out bölgeleri |
| `DriveConstants` | Swerve kinematik, motor ID'leri, PID varsayılanları |
| `ShooterConstants` | RPM/Hood interpolasyon tabloları, PID, soft limit'ler |
| `IntakeConstants` | Extension/Roller PID, dişli oranı, pozisyon limitleri |
| `FeederConstants` | Indexer/Kicker PID, RPM hedefleri |
| `VisionConstants` | Kamera pozisyonları, güven eşikleri |
| `RobotMap` | CAN ID haritası |

### Yardımcı Sınıflar (`util/`)
| Dosya | Açıklama |
|---|---|
| `TunableNumber` | Kalıcı double ayar — WPILib Preferences API |
| `TunableBoolean` | Kalıcı boolean ayar — Dashboard toggle |
| `MovingShootUtil` | Hareketli atış sanal hedef hesaplayıcısı |

---

## 🛠️ Kullanım ve Kurulum

### Gereksinimler
- **WPILib 2026** geliştirme ortamı (GradleRIO 2026.2.1)
- Ek kütüphaneler: REV Robotics (REVLib 2026), CTRE Phoenix 6, AdvantageKit, Studica NavX

### Derleme ve Simülasyon
Kodu derlemek için:
```bash
./gradlew build
```

Simülasyon çalıştırmak için:
```bash
./gradlew simulateJava
```

### ⚠️ Önemli: NetworkTables Kalıcılığı
`TunableNumber` / `TunableBoolean` **varsayılan değerlerini kodda değiştirdiğinizde**, yeniden başlatmadan önce eski kalıcı dosyayı silin:
```bash
rm -f networktables.json networktables.json.bck
```
