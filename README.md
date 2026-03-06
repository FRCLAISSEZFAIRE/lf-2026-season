# 🦊 Laissez-Faire 2026

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
- [Otonom Senaryolar](#-otonom-senaryolar)
- [Kurulum](#-kurulum)

---

## 📁 Proje Yapısı

```
src/main/java/frc/robot/
├── Robot.java                    # Ana robot sınıfı
├── RobotContainer.java           # Alt sistem oluşturma ve bağlantılar
├── ControllerBindings.java       # Tek joystick buton atamaları
├── AutonomousScenarios.java      # Otonom senaryoları
├── constants/                    # Sabitler
│   ├── Constants.java            # Genel sabitler (mod, replay)
│   ├── RobotMap.java             # Motor CAN ID'leri ve PWM portları
│   ├── DriveConstants.java       # Sürüş sabitleri
│   ├── ShooterConstants.java     # Shooter sabitleri + Atış Tablosu
│   ├── ClimberConstants.java     # Tırmanma sabitleri
│   ├── FeederConstants.java      # Besleyici sabitleri
│   ├── IntakeConstants.java      # Intake sabitleri
│   ├── FieldConstants.java       # Saha pozisyonları (Hub, Pass, Climb)
│   └── VisionConstants.java      # Kamera pozisyonları ve güven eşikleri
├── subsystems/                   # Alt sistemler
│   ├── drive/                    # Custom REV MAXSwerve sürüş
│   ├── vision/                   # Çift Limelight MegaTag2 (Pose Tahmini)
│   ├── intake/                   # Alma sistemi (Kraken + NEO)
│   ├── shooter/                  # Atış sistemi (Kraken + 2x NEO)
│   ├── climber/                  # Tırmanma sistemi (2x NEO)
│   ├── feeder/                   # Besleyici (NEO, voltaj kontrollü)
│   └── led/                      # LED durum gösterimi
├── commands/                     # Komutlar
│   ├── drive/                    # Sürüş komutları (SimpleDriveToPose, TrenchPass)
│   ├── intake/                   # AutoIntakeCommand (zamanlı)
│   ├── shooter/                  # ShootCommand (akıllı atış)
│   └── climber/                  # Tırmanma komutları
└── util/                         # Yardımcı sınıflar (TunableNumber, NavGrid)
```

---

## 🤖 Alt Sistemler

Proje **Command-Based** mimari ile geliştirilmiştir. Modern vendor kütüphaneleri kullanılır (REVLib 2026, Phoenix 6).

| Alt Sistem  | Açıklama                   | Motor                     | Altyapı                          |
| ----------- | -------------------------- | ------------------------- | -------------------------------- |
| **Drive**   | Swerve sürüş (4 modül)    | SparkMax Vortex + NEO 550 | **Custom REV MAXSwerve**         |
| **Vision**  | Çift Limelight (Pose)      | -                         | **LimelightHelpers (MegaTag 2)** |
| **Intake**  | Roller + Pivot             | Kraken + NEO              | **Phoenix6 + REVLib**            |
| **Shooter** | Flywheel + Taret + Hood    | Kraken + 2x NEO           | **Phoenix6 + REVLib**            |
| **Climber** | Tırmanma Mekanizması       | 2x NEO (SparkMax)         | **REVLib 2026**                  |
| **Feeder**  | Intake → Shooter aktarımı  | NEO (SparkMax)            | **REVLib 2026 (Voltaj)**         |
| **LED**     | İttifak rengi gösterimi    | AddressableLED            | **WPILib Native**                |

### Vision Sistemi

- **Limelight 3 (Right) & Limelight 3A (Left):** Çift kamera ile **MegaTag 2** kullanılarak robot pose tahmini yapılır.
- **Sürücü Kamerası:** Intake bölgesine bakan kamera sadece video akışı sağlar (algılama yok).

### Feeder Sistemi

- Voltaj kontrollü çalışır (sensörsüz, MZ80 kaldırıldı).
- `feed()`, `reverse()`, `stop()` komutlarıyla yönetilir.

---

## 🎯 Shooter Auto-Aim

### Çalışma Prensibi

Shooter sistemi **robot konumuna göre otomatik** çalışır. `ShootCommand` sürücü sağ tetiği basılı tuttuğunda:

1. Robot pose'u alır (odometri + Limelight).
2. Hub'a olan mesafe ve açıyı hesaplar.
3. Taret, Hood ve Flywheel RPM'i otomatik ayarlar.
4. Flywheel hazır olduğunda feeder'ı çalıştırır ("Hold Fire" mantığı).

| Mod         | Koşul             | Davranış                            |
| ----------- | ----------------- | ----------------------------------- |
| **SCORING** | İttifak alanında  | Hub'a atış (Oto. Hedefleme)         |
| **FEEDING** | İttifak dışında   | Alliance Pass atışı                 |

### 📊 Atış Kalibrasyonu

Sistem **InterpolatingDoubleTreeMap** ile mesafe bazlı lineer interpolasyon yapar.

- **Hub Atışı:** 0.5m - 4.5m arası 5 kalibrasyon noktası.
- **Alliance Pass:** 4m - 8m arası 5 kalibrasyon noktası.

> **NOT:** Tüm değerler **Test Modu**'nda `Tuning/` tablosu üzerinden canlı olarak ayarlanabilir.

### Hub Pozisyon Offseti

POV tuşlarıyla hub hedef noktası kaydırılabilir (her basışta ±0.1 metre). Bu, robotun anlık kaymasına göre RPM/açı hesaplamasını ince ayar yapmak için kullanılır.

---

## 🔧 Donanım Haritası

### CAN Motor ID'leri (`RobotMap.java`)

| ID  | Motor                  | Alt Sistem |
| --- | ---------------------- | ---------- |
| 1-8 | Swerve Modülleri       | Drive      |
| 10  | Intake Roller (Kraken) | Intake     |
| 11  | Intake Pivot (NEO)     | Intake     |
| 12  | Shooter (Kraken)       | Shooter    |
| 14  | Taret (NEO)            | Shooter    |
| 15  | Hood (NEO 550)         | Shooter    |
| 16  | Feeder (NEO)           | Feeder     |
| 20  | Climber Sol (NEO)      | Climber    |
| 21  | Climber Sağ (NEO)      | Climber    |

### PWM Portları

| Port | Cihaz             |
| ---- | ----------------- |
| 9    | LED Şerit (30 LED) |

---

## 🎮 Controller Bindings (Tek Joystick)

Robotumuz **tek Xbox controller** ile kontrol edilir. Operatör joystick'i yoktur.

| Buton                | Aksiyon                                         |
| -------------------- | ----------------------------------------------- |
| **Sol Joystick**     | İleri/Geri, Sol/Sağ hareket                     |
| **Sağ Joystick X**   | Dönme                                           |
| **Sağ Tetik**        | **Akıllı Atış** (Pose tabanlı, basılı tut)      |
| **Sol Tetik**        | **Intake Toggle** (aç/kapat)                    |
| **Sağ Bumper**       | **TrenchPass** (A↔B ittifak geçişi, basılı tut) |
| **A**                | **Climber Uzat** (yukarı, basılı tut)            |
| **B**                | **Climber Çek** (aşağı, basılı tut)              |
| **X**                | **Kusma** (intake + feeder ters, basılı tut)     |
| **Y**                | **Tower'a Sürüş** (otomatik hizalama, basılı tut)|
| **POV ↑/↓**          | Hub Y offset ±0.1m                              |
| **POV ←/→**          | Hub X offset ±0.1m                              |

---

## 🎛️ Dashboard Kontrolleri

Tüm alt sistemler **NetworkTables** üzerinden `Tuning/` tablosu altında canlı olarak ayarlanabilir.

### Nasıl Kullanılır?

1. Robotu **Test Modu**'na alın (Driver Station).
2. Shuffleboard, Glass veya Elastic Dashboard üzerinden `Tuning` tablosunu açın.
3. Değerleri değiştirin — robot anlık tepki verir.

### Tablo Yapısı

| Tablo Yolu | Açıklama |
| ---------- | -------- |
| `Tuning/Shooter/Hub/` | Hub atış kalibrasyonu (RPM/Hood @ Mesafe) |
| `Tuning/Shooter/ToAlliance/` | Alliance Pass kalibrasyonu |
| `Tuning/Shooter/Aiming/` | Canlı hedefleme verileri (mesafe, açı, hub offset) |
| `Tuning/Shooter/Offsets/` | Taret, Hood, Flywheel offset değerleri |
| `Tuning/Auto/` | Otonom süre ayarları (IntakeTimeout, ShootTimeout) |
| `Tuning/Drive/` | Sürüş PID değerleri |
| `Tuning/Climber/` | Tırmanma pozisyonları ve PID |

---

## 🤖 Otonom Senaryolar

Otonom senaryolar `AutonomousScenarios.java` içinde tanımlanır. Dashboard'dan `SendableChooser` ile seçilir.

- **AutoIntakeCommand:** Zamanlı çalışır, belirtilen süre boyunca intake + feeder çalıştırır.
- **ShootCommand:** Pose tabanlı akıllı atış (hub veya pass otomatik seçilir).
- **TrenchPassCommand:** İttifak geçiş noktaları arasında otomatik sürüş.

Süre parametreleri Dashboard'dan ayarlanabilir:
- `Tuning/Auto/IntakeTimeout` — Intake çalışma süresi (saniye)
- `Tuning/Auto/FullShootTimeout` — Tam atış süresi
- `Tuning/Auto/StartShootTimeout` — Başlangıç atış süresi

---

## 🚀 Kurulum

### Build ve Deploy

```bash
./gradlew build        # Derleme
./gradlew deploy       # Robota yükle
./gradlew simulateJava # Simülasyon
```

### Temel Kütüphaneler

| Kütüphane | Versiyon | Kullanım |
| --------- | -------- | -------- |
| **WPILib** | 2026.2.1 | Ana framework |
| **REVLib** | 2026.x | SparkMax / NEO motor kontrolü |
| **Phoenix 6** | CTRE | Kraken / TalonFX motor kontrolü |
| **LimelightHelpers** | - | Pose tahmini (MegaTag 2) |
| **AdvantageKit** | - | Loglama ve Replay |

---

## 📊 Durum

| Özellik         | Durum                                  |
| --------------- | -------------------------------------- |
| Swerve Drive    | ✅ Tamamlandı (Custom REV/PathPlanner) |
| Vision          | ✅ Tamamlandı (Çift Limelight MegaTag2)|
| Shooter         | ✅ Tamamlandı (Pose Tabanlı Auto-Aim)  |
| Climber         | ✅ Tamamlandı                          |
| Feeder & Intake | ✅ Tamamlandı (Sensörsüz/Zamanlı)     |
| Otonom          | ✅ Entegre Edildi                      |
| Sürücü Kamerası | ✅ Sadece Video Akışı                  |

---

**FRC Team 8056 - Laissez-Faire**
