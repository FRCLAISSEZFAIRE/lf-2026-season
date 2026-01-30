# PID Ayarlama Rehberi (PID Tuning Guide)

Bu rehber, FRC robot projenizdeki PID kontrollü mekanizmaları nasıl ayarlayacağınızı açıklar.

---

## 📋 PID Nedir?

PID (Proportional-Integral-Derivative) kontrolör, bir sistemin hedef değerine ulaşmasını sağlayan geri besleme kontrol algoritmasıdır.

| Terim | Açıklama | Etki |
|-------|----------|------|
| **P (Proportional)** | Hata ile orantılı çıktı | Hızlı tepki, aşırıysa salınım |
| **I (Integral)** | Birikmiş hata | Kalıcı hataları düzeltir, aşırıysa yavaşlık |
| **D (Derivative)** | Hata değişim hızı | Salınımı azaltır, aşırıysa gürültüye duyarlı |

---

## 🔧 Ayarlama Adımları

### 1. Başlangıç Değerleri

```java
// Constants dosyanızda bu değerlerle başlayın:
public static final double kP = 0.0;
public static final double kI = 0.0;
public static final double kD = 0.0;
```

### 2. P Kazancını Ayarlayın

1. I ve D'yi 0'da tutun
2. P değerini küçük bir değerden başlatın (örn: 0.1)
3. Sistemi test edin:
   - Hedefe ulaşamıyorsa → P'yi artırın
   - Hedef etrafında salınıyorsa → P'yi azaltın
4. Hedef etrafında hafif salınım yapana kadar artırın

### 3. D Kazancını Ayarlayın

1. P ayarlıyken D'yi küçük bir değerle başlatın (örn: 0.01)
2. Salınımı azaltmak için artırın
3. Dikkat: Çok yüksek D gürültülü davranışa neden olur

### 4. I Kazancını Ayarlayın (Opsiyonel)

1. Eğer sistem hedefe tam ulaşamıyorsa I ekleyin
2. Çok küçük bir değerle başlayın (örn: 0.001)
3. Dikkat: I genellikle FRC'de gerekmez

---

## 📁 Proje Sabitleri

### IntakeConstants.java

| Sabit | Açıklama | Varsayılan |
|-------|----------|------------|
| `kPivotP` | Pivot P kazancı | 1.0 |
| `kPivotI` | Pivot I kazancı | 0.0 |
| `kPivotD` | Pivot D kazancı | 0.005 |
| `kPivotMaxVelocity` | Max hız (rad/s) | 2.0 |
| `kPivotMaxAcceleration` | Max ivme (rad/s²) | 2.0 |

### ShooterConstants.java

| Sabit | Açıklama | Varsayılan |
|-------|----------|------------|
| `kTurretDefaultP` | Turret P | 0.1 |
| `kTurretDefaultI` | Turret I | 0.0 |
| `kTurretDefaultD` | Turret D | 0.005 |
| `kHoodP` | Hood P | 2.0 |
| `kHoodI` | Hood I | 0.0 |
| `kHoodD` | Hood D | 0.05 |
| `kFlywheelP` | Flywheel P | 0.2 |

### ClimberConstants.java

| Sabit | Açıklama | Varsayılan |
|-------|----------|------------|
| `kClimberP` | Climber P | (mevcut değer) |
| `kClimberI` | Climber I | (mevcut değer) |
| `kClimberD` | Climber D | (mevcut değer) |

---

## 🎯 Mekanizma Bazlı İpuçları

### Pivot / Arm

- Başlangıç P: 1.0 - 5.0
- D: 0.01 - 0.1
- Feedforward (kG) için kol ağırlığını hesaplayın

### Flywheel

- P genellikle düşük (0.1 - 0.5)
- kS (static friction) ve kV (velocity) feedforward ekleyin
- I genellikle gerekmez

### Turret

- P: 0.05 - 0.2
- D: 0.001 - 0.01
- Continuous wrapping aktif olmalı

### Elevator / Climber

- P: 0.5 - 2.0
- kG (gravity compensation) ekleyin
- Motion profiling kullanın

---

## 📊 Telemetry ile Debug

AdvantageScope veya SmartDashboard ile:

1. **Hedef pozisyon** vs **Gerçek pozisyon** grafiği çizin
2. **Hata (error)** değerini izleyin
3. **Motor çıktısı (voltage/percent)** gözlemleyin

### Örnek Log Noktaları

```java
Logger.recordOutput("Mechanism/TargetPosition", targetPos);
Logger.recordOutput("Mechanism/ActualPosition", actualPos);
Logger.recordOutput("Mechanism/Error", targetPos - actualPos);
```

---

## ⚠️ Güvenlik Notları

1. **Düşük değerlerle başlayın** - Robot hasar görmesini önleyin
2. **Soft limitleri aktif tutun** - Mekanik hasar önleme
3. **Takım arkadaşlarını uyarın** - Test sırasında dikkatli olun
4. **Simülasyonda test edin** - Gerçek robota geçmeden önce

---

## 🔄 Hızlı Referans

```
Sistem aşırı yavaş?     → P artır
Sistem sallanıyor?      → P azalt, D artır
Hedefe tam ulaşmıyor?   → I ekle (dikkatli)
Tepki çok agresif?      → P ve D azalt
```
