# 🦊 FoxyCode 2026

**FRC 2026 Sezonu Robot Yazılımı**

![WPILib](https://img.shields.io/badge/WPILib-2026_Beta-blue)
![Java](https://img.shields.io/badge/Java-17-orange)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-Logging-green)

---

## 📋 İçindekiler

- [Proje Yapısı](#-proje-yapısı)
- [Alt Sistemler](#-alt-sistemler)
- [Donanım Haritası](#-donanım-haritası)
- [Controller Bindings](#-controller-bindings)
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
│   ├── ElevatorConstants.java    # Asansör sabitleri
│   ├── ArmConstants.java         # Kol sabitleri
│   ├── GrabberConstants.java     # Tutucu sabitleri
│   ├── ClimberConstants.java     # Tırmanıcı sabitleri
│   ├── WristConstants.java       # Bilek sabitleri
│   ├── FeederConstants.java      # Besleyici sabitleri
│   └── LEDConstants.java         # LED sabitleri
├── subsystems/                   # Alt sistemler
│   ├── drive/                    # Swerve sürüş
│   ├── vision/                   # Limelight görüş
│   ├── intake/                   # Alma sistemi
│   ├── shooter/                  # Atış sistemi
│   ├── elevator/                 # Asansör
│   ├── arm/                      # Kol
│   ├── grabber/                  # Tutucu
│   ├── climber/                  # Tırmanıcı
│   ├── wrist/                    # Bilek
│   ├── feeder/                   # Besleyici
│   └── led/                      # LED kontrolü
├── commands/                     # Komutlar
│   ├── drive/                    # Sürüş komutları
│   ├── intake/                   # Alma komutları
│   ├── shooter/                  # Atış komutları
│   ├── grabber/                  # Tutucu komutları
│   └── climber/                  # Tırmanıcı komutları
└── util/                         # Yardımcı sınıflar
    ├── TunableNumber.java        # PID ayar aracı
    └── LimelightHelpers.java     # Limelight yardımcıları
```

---

## 🤖 Alt Sistemler

| Alt Sistem | Açıklama | Motor | IO Pattern |
|------------|----------|-------|------------|
| **Drive** | Swerve sürüş (4 modül) | SparkMax Vortex + NEO 550 | ✅ |
| **Vision** | Limelight görüş sistemi | - | ✅ |
| **Intake** | Oyun parçası alma | TBD | ✅ |
| **Shooter** | Flywheel + Turret + Hood | Kraken + SparkMax | ✅ |
| **Elevator** | Dikey hareket | Kraken | ✅ |
| **Arm** | Açısal hareket | SparkMax | ✅ |
| **Grabber** | Parça tutma | SparkMax NEO 550 | ✅ |
| **Climber** | Tırmanma (2 motor) | 2x Kraken | ✅ |
| **Wrist** | Bilek döndürme | SparkMax NEO 550 | ✅ |
| **Feeder** | Intake → Shooter transfer | Falcon 500 | ✅ |
| **LED** | Durum gösterimi | AddressableLED | - |

---

## 🔧 Donanım Haritası

### CAN Motor ID'leri

| ID | Motor | Alt Sistem |
|----|-------|------------|
| 1-2 | Front Left Drive/Turn | Drive |
| 3-4 | Front Right Drive/Turn | Drive |
| 5-6 | Rear Left Drive/Turn | Drive |
| 7-8 | Rear Right Drive/Turn | Drive |
| 10 | Intake | Intake |
| 11-12 | Shooter Master/Follower | Shooter |
| 13 | Turret | Shooter |
| 14 | Hood | Shooter |
| 20 | Elevator | Elevator |
| 21 | Arm | Arm |
| 22 | Grabber | Grabber |
| 23-24 | Climber Left/Right | Climber |
| 25 | Wrist | Wrist |
| 26 | Feeder | Feeder |

### DIO Portları

| Port | Sensör | Açıklama |
|------|--------|----------|
| 0 | MZ80 | Intake sensörü |
| 1 | MZ80 | Shooter sensörü |

### PWM Portları

| Port | Cihaz | Açıklama |
|------|-------|----------|
| 0 | AddressableLED | 60 LED strip |

---

## 🎮 Controller Bindings

### Driver Controller (Port 0)

| Buton | Aksiyon |
|-------|---------|
| Sol Joystick | İleri/Geri, Sol/Sağ hareket |
| Sağ Joystick | Dönme |
| A | Gyro sıfırlama |
| X | X-Stance (fren) |
| B | LED Rainbow |

### Operator Controller (Port 1)

| Buton | Aksiyon |
|-------|---------|
| Sağ Tetik | Intake + Feeder (alma) |
| Sol Tetik | Intake + Feeder ters (çıkarma) |
| Sağ Bumper | Atış |
| D-Pad ↑ | Elevator yukarı |
| D-Pad ↓ | Elevator aşağı |
| D-Pad → | Arm yukarı |
| D-Pad ← | Arm aşağı |
| A | Grabber kapat (tut) |
| B | Grabber aç (bırak) |
| X | Wrist merkez |
| Y | Wrist skor pozisyonu |
| LB + Sağ Stick ↑ | Tırman yukarı |
| LB + Sağ Stick ↓ | Tırman aşağı |
| Start | Acil durdur |

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

### Tuning

`Constants.tuningMode = true` yaparak Shuffleboard üzerinden PID ayarı yapılabilir.

---

## 📊 Durum

| Özellik | Durum |
|---------|-------|
| Swerve Drive | ✅ Tamamlandı |
| Vision Integration | ✅ Tamamlandı |
| Elevator | ✅ Tamamlandı |
| Arm | ✅ Tamamlandı |
| Grabber | ✅ Tamamlandı |
| Shooter | ✅ Tamamlandı |
| Climber | ✅ Tamamlandı |
| Wrist | ✅ Tamamlandı |
| Feeder | ✅ Tamamlandı |
| LED | ✅ Tamamlandı |
| Autonomous | 🔲 Bekleniyor |
| PathPlanner | 🔲 Bekleniyor |

---

## 👥 Takım

**FRC Team [8056] - FoxyCode**

---

## 📄 Lisans

Bu proje WPILib BSD lisansı altındadır.
