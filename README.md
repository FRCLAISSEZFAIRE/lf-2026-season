# 🦊 FoxyCode 2026

**FRC 2026 REBUILT Sezonu Robot Yazılımı**

> 🏗️ **REBUILT** - FIRST AGE temalı arkeoloji sezonunun FRC oyunu. Fuel (köpük toplar) Hub'a atılır, Tower'a tırmanılır.

![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)
![Java](https://img.shields.io/badge/Java-17-orange)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-Logging-green)
![Game](https://img.shields.io/badge/Game-REBUILT-red)

---

## 📋 İçindekiler

- [Proje Yapısı](#-proje-yapısı)
- [Alt Sistemler](#-alt-sistemler)
- [Shooter Auto-Aim](#-shooter-auto-aim)
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
│   ├── ShooterConstants.java     # Shooter sabitleri + Shooting Table
│   ├── ClimberConstants.java     # Climber (Tırmanma) sabitleri
│   └── FieldConstants.java       # Saha pozisyonları
├── subsystems/                   # Alt sistemler
│   ├── drive/                    # Swerve sürüş
│   ├── vision/                   # Limelight görüş
│   ├── intake/                   # Alma sistemi
│   ├── shooter/                  # Atış sistemi (Turret + Hood + Flywheel)
│   ├── climber/                  # Climber (Tırmanma)
│   ├── feeder/                   # Besleyici
│   └── led/                      # LED kontrolü
├── commands/                     # Komutlar
└── util/                         # Yardımcı sınıflar
```

---

## 🤖 Alt Sistemler

| Alt Sistem  | Açıklama                  | Motor                     | IO Pattern |
| ----------- | ------------------------- | ------------------------- | ---------- |
| **Drive**   | Swerve sürüş (4 modül)    | SparkMax Vortex + NEO 550 | ✅         |
| **Vision**  | Limelight 3 (Pose) + 3A   | -                         | ✅         |
| **Intake**  | Roller + Pivot (Active)   | 2x NEO (SparkMax)         | ✅         |
| **Shooter** | Flywheel + Turret + Hood  | Kraken + NEO + NEO550     | ✅         |
| **Climber** | Tırmanma Mekanizması      | 2x Kraken                 | ✅         |
| **Feeder**  | Intake → Shooter transfer | NEO (SparkMax)            | ✅         |
| **LED**     | Durum gösterimi           | AddressableLED            | -          |

---

## 🎯 Shooter Auto-Aim

### Çalışma Modları

Shooter sistemi robot konumuna göre **otomatik** çalışır:

| Mod         | Koşul             | Davranış                          |
| ----------- | ----------------- | --------------------------------- |
| **IDLE**    | Butona basılmamış | Flywheel idle (2000 RPM)          |
| **SCORING** | İttifak alanında  | Hub'a tam güç atış                |
| **FEEDING** | İttifak dışında   | İttifak alanına top besleme (lob) |

### Alliance Zone (İttifak Alanı)

- **Blue Alliance:** X < 4.0m (sol taraf)
- **Red Alliance:** X > 12.49m (sağ taraf)

Robot ittifak alanındayken **Hub'a** nişan alır. Dışındayken **ittifak alanına** top besler.

### 📊 Shooting Parameters

Sadece **CLOSE** ve **FAR** noktası tanımlı, ara değerler **otomatik lineer interpolasyon** ile hesaplanır:

| Parametre  | CLOSE (Yakın) | FAR (Uzak) |
| ---------- | ------------- | ---------- |
| Mesafe     | 1.5 m         | 7.0 m      |
| Hood Açısı | 55° (dik)     | 17° (düz)  |
| Flywheel   | 4500 RPM      | 7000 RPM   |

> **Tuning:** Sadece 6 değeri ayarlaman yeterli! `ShooterConstants.java` içindeki `kClose*` ve `kFar*` değerlerini değiştir.

### Feeding Mode (Top Besleme)

İttifak alanı dışında:

- **Flywheel:** 3500 RPM (düşük güç)
- **Hood Açısı:** 50° (lob atış)

---

## 🔧 Donanım Haritası

### CAN Motor ID'leri (`RobotMap.java`)

| ID  | Motor                  | Alt Sistem |
| --- | ---------------------- | ---------- |
| 1-2 | Front Left Drive/Turn  | Drive      |
| 3-4 | Front Right Drive/Turn | Drive      |
| 5-6 | Rear Left Drive/Turn   | Drive      |
| 7-8 | Rear Right Drive/Turn  | Drive      |
| 10  | Intake Roller          | Intake     |
| 11  | Intake Pivot           | Intake     |
| 12  | Shooter Flywheel       | Shooter    |
| 13  | Shooter Follower       | Shooter    |
| 14  | Turret                 | Shooter    |
| 15  | Hood (NEO 550)         | Shooter    |
| 16  | Feeder                 | Feeder     |
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

| Buton                | Aksiyon                                |
| -------------------- | -------------------------------------- |
| **Sol Joystick**     | İleri/Geri, Sol/Sağ hareket            |
| **Sağ Joystick X**   | Dönme                                  |
| **X**                | **Auto Intake** (Kamera takibi + Alma) |
| **Y**                | **Auto Aim** (Robot hedefe döner)      |
| **POV Yukarı/Aşağı** | Climb Pozisyonu Seçimi                 |
| **POV Sağ/Sol**      | Source Seçimi                          |

### Operator Controller (Port 1)

| Buton                | Aksiyon                             |
| -------------------- | ----------------------------------- |
| **Sağ Tetik**        | Intake + Feeder (Normal Alma)       |
| **Sol Tetik**        | Sadece Intake Ters (Kusma)          |
| **Sağ Bumper**       | Atış (Auto-Aim devreye girer)       |
| **Sol Bumper**       | Flywheel Ters (Sıkışma Giderme)     |
| **X**                | Auto Intake                         |
| **Y**                | Auto Aim & Shoot                    |
| **A**                | Climb Retract (Robotu Yukarı Çeker) |
| **B**                | Climb Extend (Kancaları Uzatır)     |
| **POV Sağ/Sol**      | Climb Position Seçimi               |
| **POV Yukarı/Aşağı** | Manuel Tırmanma                     |
| **Start**            | ACİL DURDUR                         |

---

## 🖥️ Dashboard Kontrolleri

- **Climb Position:** Otomatik tırmanma için hedef kule pozisyonu
- **START AUTO CLIMB:** Otomatik tırmanma sekansını başlatır

### AdvantageScope Logs

```
Shooter/Mode           → IDLE / SCORING / FEEDING
Shooter/InAllianceZone → true / false
Shooter/DistanceToTarget → mesafe (m)
Shooter/HoodTarget     → hedef açı (°)
Shooter/FlywheelTargetRPM → hedef hız
Shooter/IsReadyToShoot → hazır mı?
```

---

## 🚀 Kurulum

### Gereksinimler

- WPILib 2026
- Java 17+
- Gradle 8.x

### Build

```bash
./gradlew build        # Build
./gradlew deploy       # Deploy to robot
./gradlew simulateJava # Simulate
```

### Bağımlılıklar

Otomatik olarak `vendordeps/` klasöründen yüklenir:

- CTRE Phoenix 6
- REVLib 2026
- NavX (Studica)
- AdvantageKit

---

## 💻 Geliştirme

### IO Pattern

```
subsystem/
├── SubsystemIO.java         # Interface + AutoLog
├── SubsystemIOReal.java     # Gerçek robot
├── SubsystemIOSim.java      # Simülasyon
└── SubsystemSubsystem.java  # Ana sınıf
```

### PathPlanner

- **Pathfinding:** Dinamik hedef seçimi
- **NamedCommands:** `AutoIntake`, `Shoot`, `ClimberExtend`, `ClimberRetract`

---

## 📊 Durum

| Özellik            | Durum                               |
| ------------------ | ----------------------------------- |
| Swerve Drive       | ✅ Tamamlandı                       |
| Vision Integration | ✅ Tamamlandı                       |
| Shooter Auto-Aim   | ✅ Tamamlandı                       |
| Climber            | ✅ Tamamlandı (Auto Climb + Sensor) |
| Feeder             | ✅ Tamamlandı                       |
| LED                | ✅ Tamamlandı                       |
| Autonomous         | 🚧 Entegre Edildi                   |
| PathPlanner        | 🚧 Entegre Edildi                   |

---

## 👥 Takım

**FRC Team [8056] - FoxyCode**

---

## 📄 Lisans

Bu proje WPILib BSD lisansı altındadır.
