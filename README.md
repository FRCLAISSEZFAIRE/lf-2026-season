# 🦊 FoxyCode 2026

**FRC 2026 REBUILT Sezonu Robot Yazılımı**

> 🏗️ **REBUILT** - FIRST AGE temalı arkeoloji sezonunun FRC oyunu. Fuel (köpük toplar) Hub'a atılır, Tower'a tırmanılır.

![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)
![Java](https://img.shields.io/badge/Java-17-orange)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-Logging-green)
![Game](https://img.shields.io/badge/Game-REBUILT-red)
![YAGSL](https://img.shields.io/badge/YAGSL-2026.1.20-yellow)

---

## 📋 İçindekiler

- [Proje Yapısı](#-proje-yapısı)
- [Alt Sistemler](#-alt-sistemler)
- [Shooter Auto-Aim](#-shooter-auto-aim)
- [Donanım Haritası](#-donanım-haritası)
- [Controller Bindings](#-controller-bindings)
- [Dashboard Kontrolleri](#-dashboard-kontrolleri)
- [Kurulum](#-kurulum)

---

## 📁 Proje Yapısı

```
src/main/java/frc/robot/
├── Robot.java                    # Ana robot sınıfı
├── RobotContainer.java           # Subsystem ve wiring
├── ControllerBindings.java       # Tüm controller bindings
├── constants/                    # Sabitler
│   ├── Constants.java            # Genel sabitler
│   ├── RobotMap.java             # Motor ve sensör ID'leri
│   ├── DriveConstants.java       # Sürüş sabitleri
│   ├── ShooterConstants.java     # Shooter sabitleri + Shooting Table
│   ├── ClimberConstants.java     # Climber (Tırmanma) sabitleri
│   ├── FeederConstants.java      # Feeder sabitleri
│   ├── IntakeConstants.java      # Intake sabitleri
│   └── FieldConstants.java       # Saha pozisyonları
├── subsystems/                   # Alt sistemler
│   ├── drive/                    # YAGSL tabanlı Swerve sürüş
│   ├── vision/                   # Limelight MegaTag2 + Detector
│   ├── intake/                   # Alma sistemi (Phoenix6 + REVLib)
│   ├── shooter/                  # Atış sistemi (Phoenix6 + REVLib)
│   ├── climber/                  # Climber sistemi (REVLib)
│   ├── feeder/                   # Besleyici (REVLib)
│   └── led/                      # LED kontrolü
├── commands/                     # Komutlar
└── util/                         # Yardımcı sınıflar
```

---

## 🤖 Alt Sistemler

Proje **Command-Based** mimari ile geliştirilmiştir ve modern vendor kütüphanelerini kullanır (REVLib 2026, Phoenix 6).

| Alt Sistem  | Açıklama                  | Motor                     | Altyapı |
| ----------- | ------------------------- | ------------------------- | ------- |
| **Drive**   | Swerve sürüş (4 modül)    | SparkMax Vortex + NEO 550 | **YAGSL** |
| **Vision**  | LL3 (Pose) + LL3A (GamePiece) | -                     | **Native** |
| **Intake**  | Roller (Vel) + Pivot (Pos)| Kraken + NEO              | **Phoenix6 + REVLib** |
| **Shooter** | Flywheel + Turret + Hood  | Kraken + 2x NEO           | **Phoenix6 + REVLib** |
| **Climber** | Tırmanma Mekanizması      | 2x NEO (SparkMax)         | **REVLib 2026** |
| **Feeder**  | Intake → Shooter transfer | NEO (SparkMax)            | **REVLib 2026** |
| **LED**     | Durum gösterimi           | AddressableLED            | **Native** |

### Vision Sistemi
- **Limelight 3 (limelight):** MegaTag 2 teknolojisi ile hassas robot pose estimation (NavX sync ile).
- **Limelight 3A (limelight-3a):** Neural network ile Game Piece algılama (Auto-Intake için).

---

## 🎯 Shooter Auto-Aim

### Çalışma Modları

Shooter sistemi robot konumuna göre **otomatik** çalışır:

| Mod         | Koşul             | Davranış                          |
| ----------- | ----------------- | --------------------------------- |
| **IDLE**    | Butona basılmamış | Flywheel idle (2000 RPM)          |
| **SCORING** | İttifak alanında  | Hub'a tam güç atış                |
| **FEEDING** | İttifak dışında   | İttifak alanına top besleme (lob) |

### 📊 Shooting Parameters

Sadece **CLOSE** ve **FAR** noktası tanımlı, ara değerler **otomatik lineer interpolasyon** ile hesaplanır:

| Parametre  | CLOSE (Yakın) | FAR (Uzak) |
| ---------- | ------------- | ---------- |
| Mesafe     | 1.5 m         | 7.0 m      |
| Hood Açısı | 55° (dik)     | 17° (düz)  |
| Flywheel   | 4500 RPM      | 7000 RPM   |

---

## 🔧 Donanım Haritası

### CAN Motor ID'leri (`RobotMap.java`)

| ID  | Motor                  | Alt Sistem |
| --- | ---------------------- | ---------- |
| 1-8 | Swerve Modules         | Drive      |
| 10  | Intake Roller (Kraken) | Intake     |
| 11  | Intake Pivot (NEO)     | Intake     |
| 12  | Shooter (Kraken)| Shooter    |
| 14  | Turret (NEO)           | Shooter    |
| 15  | Hood (NEO 550)         | Shooter    |
| 16  | Feeder (NEO)           | Feeder     |
| 20  | Climber Left (NEO)     | Climber    |
| 21  | Climber Right (NEO)    | Climber    |

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
| **A**                | **ATIŞ** (Shooting + LED Animasyon) |
| **Sağ Tetik**        | Intake + Feeder (Normal Alma)       |
| **Sol Tetik**        | Sadece Intake Ters (Kusma)          |
| **Sağ Bumper**       | Hazırlık (Prep Shot)                |
| **Sol Bumper**       | Flywheel Ters (Sıkışma Giderme)     |
| **X**                | Climb Retract (Robotu Yukarı Çeker) |
| **Y**                | Climb Extend (Kancaları Uzatır)     |
| **Start**            | ACİL DURDUR                         |

---

## 🚀 Kurulum

### Build

```bash
./gradlew build        # Build
./gradlew deploy       # Deploy to robot
./gradlew simulateJava # Simulate
```

### Temel Kütüphaneler

- **YAGSL (Yet Another Generic Swerve Library):** 2026.1.20
- **REVLib:** 2026.x (SparkMax / NEO)
- **Phoenix 6:** CTRE (Kraken / TalonFX)
- **AdvantageKit:** Logging & Replay

---

## 📊 Durum

| Özellik | Durum |
| --- | --- |
| Swerve Drive | ✅ Tamamlandı (YAGSL 2026) |
| Vision | ✅ Tamamlandı (MT2 + Object Det.) |
| Shooter | ✅ Tamamlandı (Modernized) |
| Climber | ✅ Tamamlandı (Modernized) |
| Feeder & Intake | ✅ Tamamlandı (Modernized) |
| Otonom | 🚧 Entegre Edildi |

---

**FRC Team [8056] - FoxyCode**
