# Otonom (PathPlanner) Komutları Rehberi

Bu belge, PathPlanner otonom rutinlerinde kullanılmak üzere hazırlanan komutların ne işe yaradığını ve nasıl çalıştığını açıklamaktadır. Ayrıca `.auto` dosyalarınızda "Named Commands" eventleri kullanarak bu komutları tetikleyebilirsiniz.

## 1. `autoIntake`
**Amacı:** Limelight 3A kullanarak yerdeki oyun objesini (game piece) otomatik olarak algılar ve toplar.
**Çalışma Mantığı:**
- Robotu hedefe yönlendirmek için görüntü işleme (Vision Yaw) kullanır ve PID ile hizalar.
- Hedefe kilitlendiğinde robot yavaşça ileri sürülür ve aynı anda Intake (alma) ile Feeder (besleme) sistemini çalıştırır.
- Eğer hedef kameradan çıkarsa (kaybolursa), ayarlanmış bir süre boyunca aramaya devam eder (timeout süreci).
- Feeder tam dolduğunda (oyun objesi hazneye oturduğunda) veya timeout süresi dolduğunda otomatik olarak durur.
- **Simülasyon Modu (Güncel):** Sensör eksikliğinden dolayı sonsuza kadar çalışmaması için `autoIntake` komutu şu an manuel olarak 12 Voltta 5 saniye boyunca çalışıp kendi kendine kapanacak şekilde (körlemesine) otonom simülasyonları için sınırlandırılmıştır.

## 2. `autoShoot`
**Amacı:** Robotun sahadaki anlık konumunu (odometri) kullanarak hedefe otomatik nişan alıp atış yapar.
**Çalışma Mantığı:**
- Sürekli olarak Taret, Hood (üst kapak) ve Flywheel (volan hızı) değerlerini robotun anlık (Pose) pozisyonuna göre günceller.
- Atış komutuna basıldığı andan itibaren önce nişan alır. Sadece Flywheel hedef RPM hızına ulaştığında ve hazır olduğunda Feeder çalışarak oyun objesini fırlatır.
- Özel durum: Eğer robot İttifak Bölgesi (Alliance Zone) içindeyse doğrudan Skora (Hub) atış yapar. Dışındaysa takım arkadaşlarına pas verme moduna geçer.
- "Hold Fire" mantığıyla nişan hassasiyeti bozulursa beslemeyi geçici olarak durdurur ve doğruluğu korur.
- **Simülasyon Modu (Güncel):** Simülasyon testlerinde (hedef bulamama takılmalarını önlemek için) 5 saniye boyunca 3000 RPM 30° Hood ve 0° Taret açılarına zorlanmış, 5 saniye sonunda otomatik duran temel bir atış rutinine çevrilmiştir.

## 3. `autoClimb`
**Amacı:** Otomatik Tırmanma Komutu. Otonom evresinde veya yarı otonom olarak asılma (climb) bölgesine kendi kendini sürer ve kancaları geçirip havaya kalkar.
**Çalışma Mantığı:**
- Hedef noktasını doğrudan `FieldConstants.getTowerCenter()` üzerinden alarak kulenin ortasına pathfinding işlemi başlatır.
- Asılma noktasına güvenli bir hızla vardıktan sonra `ClimberExtendCommand` ile kancaları 1. Seviye bara uzatır.
- Çok kısa bir süre ileri atılarak kancaların tırmanma borusuna kilitlenmesini güvenceye alır.
- Ardından kancaları geri çekerek (`ClimberRetractCommand`) robotun yerden kesilmesini ve kilitlenme pozisyonuna (1. Seviye asılma) oturmasını sağlar. Çıtçıt/Koltuk (Seat sensor) sensörü asılmayı doğruladığında komut biter.

## 4. `ShootToAllianceCommand`
**Amacı:** Oyun objesini skora değil, spesifik olarak ittifak besleme istasyonuna / pas bölgesine göndermek (pas vermek) için kullanılır.
**Çalışma Mantığı:**
- `ShootCommand` ile aynı temel mantığa dayanır ancak hesaplanan hedef skor alanı (Hub) değil, pas verme alanıdır.
- Sadece atışa tamamen hazır olunduğunda Feeder aktifleşerek pası atar.

## 5. `autoPass` (TrenchPassCommand)
**Amacı:** Robotun sahadaki dar geçiş bölgelerinden (Trench) veya diğer noktalardan otonom bir şekilde güvenle karşıya geçmesini sağlar.
**Çalışma Mantığı:**
- Robotun anlık bulunduğu konumu (üst yarıda mı alt yarıda mı) algılar.
- Bulunduğu ittifaka (Kırmızı/Mavi) göre geçiş hedeflerini (A'dan B'ye veya C'den D'ye gibi) dinamik olarak belirleyip ardışık rotalar çizer.
- Geçiş sırasında darbe riskini veya robot yüksekliğini düşürmek için komut çalışır çalışmaz Hood açısını 0'a çekerek sistemi kapatır/güvenliğe alır.

---
**💡 PathPlanner Entegrasyon Notu:**
Yukarıdaki komutları kullanabilmek için `RobotContainer.java` içerisindeki `NamedCommands.registerCommand("KomutIsmi", new Komut(..))` bölümünde tanımlamaların yapılmış olması gereklidir (halihazırda yapılmıştır). Bu sayede PathPlanner uygulamasındaki Event markerlara (örn: `autoShoot`, `autoIntake`, `autoClimb`) bu isimleri yazarak tetikleyebilirsiniz.
