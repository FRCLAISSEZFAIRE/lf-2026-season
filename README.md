# 🦊 FoxyCode 2026

**FRC 2026 REBUILT Sezonu Robot Yazılımı**

> 🏗️ **REBUILT** - FIRST AGE temalı arkeoloji sezonunun FRC oyunu. Fuel (köpük toplar) Hub'a atılır, Tower'a tırmanılır.

![WPILib](https://img.shields.io/badge/WPILib-2026.1.1-blue)
![Java](https://img.shields.io/badge/Java-17-orange)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-Logging-green)
![Game](https://img.shields.io/badge/Game-REBUILT-red)

---

## 📋 İçindekiler

- [Proje Yapısı](#-proje-yapısı)
- [Alt Sistemler](#-alt-sistemler)
- [Donanım Haritası](#-donanım-haritası)
- [Controller Bindings](#-controller-bindings)
- [Dashboard Kontrolleri](#-dashboard-kontrolleri)
- [Kurulum](#-kurulum)
- [Geliştirme](#-geliştirme)

---

## 📁 Proje Yapısı

```
src/main/java/frc/robot/
├── Robot.java                    # Ana robot sınıfı
├── RobotContainer.java           # Subsystem ve IO wiring
├── ControllerBindings.java       # Tüm controller bindings
├── constants/                    # Sabitler
│   ├── Constants.java            # Genel sabitler
│   ├── RobotMap.java             # Motor ve sensör ID'leri
│   ├── DriveConstants.java       # Sürüş sabitleri
│   ├── ClimberConstants.java     # Climber (Tırmanma) sabitleri
│   └── FieldConstants.java       # Saha ve skor pozisyonları
├── subsystems/                   # Alt sistemler
│   ├── drive/                    # Swerve sürüş
│   ├── vision/                   # Limelight görüş (VisionSubsystem)
│   ├── intake/                   # Alma sistemi
│   ├── shooter/                  # Atış sistemi
│   ├── climber/                  # Climber (Tırmanma)
│   ├── feeder/                   # Besleyici
│   └── led/                      # LED kontrolü
├── commands/                     # Komutlar
│   ├── drive/                    # Sürüş komutları
│   └── intake/                   # Alma komutları
└── util/                         # Yardımcı sınıflar
    └── LimelightHelpers.java     # Limelight yardımcıları
```

---

## 🤖 Alt Sistemler

| Alt Sistem  | Açıklama                  | Motor                     | IO Pattern |
| ----------- | ------------------------- | ------------------------- | ---------- |
| **Drive**   | Swerve sürüş (4 modül)    | SparkMax Vortex + NEO 550 | ✅         |
| **Vision**  | Limelight 3 (Pose) + 3A   | -                         | ✅         |
| **Intake**  | Roller + Pivot (Active)   | 2x NEO (SparkMax)         | ✅         |
| **Shooter** | Flywheel + Turret + Hood  | Kraken + NEO + Servos     | ✅         |
| **Climber** | Tırmanma Mekanizması      | 2x Kraken                 | ✅         |
| **Feeder**  | Intake → Shooter transfer | NEO (SparkMax)            | ✅         |
| **LED**     | Durum gösterimi           | AddressableLED            | -          |

> **Climber:** Otomatik tırmanma (Auto Climb) özelliği ve sensör tabanlı (Seat Sensor) kilitleme sistemi mevcuttur.

---

## 🔧 Donanım Haritası

### CAN Motor ID'leri

| ID  | Motor                  | Alt Sistem |
| --- | ---------------------- | ---------- |
| 1-2 | Front Left Drive/Turn  | Drive      |
| 3-4 | Front Right Drive/Turn | Drive      |
| 5-6 | Rear Left Drive/Turn   | Drive      |
| 7-8 | Rear Right Drive/Turn  | Drive      |
| 10  | Intake Roller          | Intake     |
| 11  | Intake Pivot           | Intake     |
| 12  | Shooter Master         | Shooter    |
| 13  | Shooter Follower       | Shooter    |
| 14  | Feeder                 | Feeder     |
| 15  | Turret                 | Shooter    |
| 20  | Climber Left           | Climber    |
| 21  | Climber Right          | Climber    |

### DIO Portları

| Port | Sensör       | Açıklama                               |
| ---- | ------------ | -------------------------------------- |
| 0    | MZ80         | **Feeder Bottom**                      |
| 1    | MZ80         | **Feeder Top**                         |
| 4    | Limit Switch | **Seat Sensor** (Climber Kanca Kilidi) |

### PWM Portları

| Port | Cihaz          | Açıklama     |
| ---- | -------------- | ------------ |
| 0    | AddressableLED | 60 LED strip |

---

## 🎮 Controller Bindings

### Driver Controller (Port 0)

| Buton                | Aksiyon                                         |
| -------------------- | ----------------------------------------------- |
| **Sol Joystick**     | İleri/Geri, Sol/Sağ hareket                     |
| **Sağ Joystick X**   | Dönme                                           |
| **X**                | **Auto Intake** (Kamera takibi + Alma)          |
| **Y**                | **Auto Aim** (Robotu Shooter hedefine döndürür) |
| **POV Yukarı/Aşağı** | Skor Hedefi Seçimi (Hub Pozisyonları)           |
| **POV Sağ/Sol**      | Kaynak (Source) Seçimi                          |

### Operator Controller (Port 1)

| Buton                | Aksiyon                                                  |
| -------------------- | -------------------------------------------------------- |
| **Sağ Tetik**        | Intake + Feeder (Normal Alma)                            |
| **Sol Tetik**        | **Sadece Intake Ters** (Kusma - Feeder Çalışmaz)         |
| **Sağ Bumper**       | Atış (Manuel / Yakın Mesafe)                             |
| **Sol Bumper**       | **Flywheel Ters** (Sıkışma Giderme)                      |
| **X**                | **Auto Intake** (Kamera Takibi + Alma)                   |
| **Y**                | **Auto Aim & Shoot** (Robotu Döndür + Otomatik Atış)     |
| **A**                | **Climb Retract** (Robotu Yukarı Çeker - Sensörle Durur) |
| **B**                | **Climb Extend** (Kancaları Uzatır)                      |
| **POV Sağ/Sol**      | **Climb Position** Seçimi (Tower Left/Mid/Right)         |
| **POV Yukarı/Aşağı** | Manuel Tırmanma (Yukarı/Aşağı)                           |
| **Start**            | **ACİL DURDUR** (Tüm sistemleri durdurur, LED: Error)    |

---

## 🖥️ Dashboard Kontrolleri

SmartDashboard üzerinde aşağıdaki kontroller mevcuttur:

- **Climb Position:** Otomatik tırmanma için hedef kule pozisyonu seçimi (`Tower Left`, `Tower Mid`, `Tower Right`). Operator POV tuşları ile de değiştirilebilir.
- **START AUTO CLIMB:** Otomatik tırmanma sekansını başlatır. (Pathfind -> Extend -> Hizala -> Retract -> Wait for Sensor).

---

## 🚀 Kurulum

### Gereksinimler

- WPILib 2026 Beta
- Java 17+
- Gradle 8.x

### Build

```bash
# Build
./gradlew build

# Deploy to robot
./gradlew deploy

# Simulate
./gradlew simulateJava
```

### Bağımlılıklar

Otomatik olarak `vendordeps/` klasöründen yüklenir:

- CTRE Phoenix 6
- REVLib 2026 Beta
- NavX (Studica)
- AdvantageKit

---

## 💻 Geliştirme

### IO Pattern

Her alt sistem şu yapıyı takip eder:

```
subsystem/
├── SubsystemIO.java         # Interface + AutoLog
├── SubsystemIOReal.java     # Gerçek robot implementasyonu
├── SubsystemIOSim.java      # Simülasyon implementasyonu
└── SubsystemSubsystem.java  # Ana subsystem sınıfı
```

### Logging

AdvantageKit ile tüm veriler AdvantageScope'ta görüntülenebilir:

```java
Logger.recordOutput("Subsystem/Value", value);
```

### PathPlanner

Proje **AutoBuilder** ile yapılandırılmıştır.

- **Pathfinding:** Driver Controller üzerinden dinamik hedef seçimi mümkündür.
- **NamedCommands:** `AutoIntake`, `Shoot`, `ClimberExtend`, `ClimberRetract` vb. komutlar GUI üzerinden kullanılabilir.

---

## 📊 Durum

| Özellik            | Durum                               |
| ------------------ | ----------------------------------- |
| Swerve Drive       | ✅ Tamamlandı                       |
| Vision Integration | ✅ Tamamlandı                       |
| Climber            | ✅ Tamamlandı (Auto Climb + Sensor) |
| Shooter            | ✅ Tamamlandı                       |
| Feeder             | ✅ Tamamlandı                       |
| LED                | ✅ Tamamlandı                       |
| Autonomous         | 🚧 Entegre Edildi (AutoBuilder)     |
| PathPlanner        | 🚧 Entegre Edildi                   |

---

## 👥 Takım

**FRC Team [8056] - FoxyCode**

---

## 📄 Lisans

Bu proje WPILib BSD lisansı altındadır.
