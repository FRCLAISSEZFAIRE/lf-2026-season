package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Hareket halinde atış (Shoot-on-the-Move) için fizik hesaplamaları.
 * Physics calculations for shooting while moving.
 *
 * <h2>Fizik Modeli / Physics Model:</h2>
 * <ol>
 *   <li>2-iterasyonlu sanal hedef yakınsaması (Virtual Target Convergence)</li>
 *   <li>Radyal hız telafisi ile efektif mesafe (Effective Distance via Radial Velocity)</li>
 *   <li>Mekanik gecikme telafisi (Mechanical Latency Compensation)</li>
 *   <li>Taret açısal öncüleme (Turret Angular Lead)</li>
 * </ol>
 *
 * Tüm hesaplamalar saha-referanslı (inertial) koordinat sisteminde yapılır.
 * All calculations are in the field-relative (inertial) reference frame.
 */
public final class MovingShootUtil {

    private MovingShootUtil() {}

    // =========================================================================
    // RESULT RECORD
    // =========================================================================

    /**
     * Hareket halinde atış hesabının tüm sonuçlarını taşıyan kayıt.
     * Record containing all results of a moving shoot calculation.
     *
     * @param virtualTarget       Sanal hedef pozisyonu (nişan alınacak nokta) / Virtual target position to aim at
     * @param effectiveDistance    RPM/Hood map lookup için kullanılacak efektif mesafe (m) / Effective distance for RPM/Hood map lookup
     * @param turretLeadAngleDeg  Taret öncüleme açısı (derece) / Turret lead angle (degrees)
     * @param timeOfFlight        Hesaplanan uçuş süresi (saniye) / Calculated time of flight (seconds)
     * @param radialVelocity      Hedefe doğru radyal hız bileşeni (m/s, pozitif=yaklaşma) / Radial velocity toward target (m/s, positive = approaching)
     * @param tangentialVelocity  Hedefe dik teğetsel hız bileşeni (m/s) / Tangential velocity perpendicular to target (m/s)
     */
    public record MovingShootResult(
        Translation2d virtualTarget,
        double effectiveDistance,
        double turretLeadAngleDeg,
        double timeOfFlight,
        double radialVelocity,
        double tangentialVelocity
    ) {}

    // =========================================================================
    // MAIN CALCULATION
    // =========================================================================

    /**
     * Hareket halinde atış için tam fizik hesabını gerçekleştirir.
     * Performs the complete physics calculation for shooting on the move.
     *
     * <h3>Algoritma / Algorithm:</h3>
     * <ol>
     *   <li>Gecikme telafisi: Taret pozisyonunu tahmin et / Latency comp: predict turret position</li>
     *   <li>2-iterasyonlu sanal hedef hesabı / 2-iteration virtual target calculation</li>
     *   <li>Radyal/teğetsel hız ayrıştırması / Radial/tangential velocity decomposition</li>
     *   <li>Efektif mesafe hesabı / Effective distance calculation</li>
     *   <li>Taret öncüleme açısı / Turret lead angle</li>
     * </ol>
     *
     * @param realTarget           Gerçek hedef pozisyonu (Hub merkezi) / Real target position (Hub center)
     * @param turretFieldPos       Taretin sahadaki anlık pozisyonu / Current turret position on field
     * @param fieldSpeeds          Robotun saha-referanslı hız vektörü / Robot field-relative velocity
     * @param shotSpeedMps         Topun yatay çıkış hızı (m/s) / Ball horizontal exit speed (m/s)
     * @param mechanicalLatencySec Feeder tetiklemesinden top çıkışına kadar gecikme (saniye) / Delay from feeder trigger to ball exit (seconds)
     * @param turretResponseSec    Taret PID tepki süresi (saniye) / Turret PID response time (seconds)
     * @param enableLatencyComp    Gecikme telafisi aktif mi? / Is latency compensation enabled?
     * @param enableTurretLead     Taret öncülemesi aktif mi? / Is turret lead enabled?
     * @param velocityGain         Hız kompanzasyon çarpanı / Velocity compensation gain multiplier
     *                             1.0 = fizik doğru değer / physics-correct value
     *                             0.5 = yarı kompanzasyon / half compensation
     *                             1.5 = artırılmış kompanzasyon / increased compensation
     *                             0.0 = kompanzasyon kapalı / compensation disabled
     * @return                     MovingShootResult — tüm hesaplama sonuçları / all calculation results
     */
    public static MovingShootResult calculate(
            Translation2d realTarget,
            Translation2d turretFieldPos,
            ChassisSpeeds fieldSpeeds,
            double shotSpeedMps,
            double mechanicalLatencySec,
            double turretResponseSec,
            boolean enableLatencyComp,
            boolean enableTurretLead,
            double velocityGain) {

        // Güvenlik: Atış hızı çok düşükse statik sonuç döndür
        // Safety: if shot speed is too low, return static result
        if (shotSpeedMps < 0.1) {
            double staticDist = turretFieldPos.getDistance(realTarget);
            return new MovingShootResult(realTarget, staticDist, 0, 0, 0, 0);
        }

        // Hız gain çarpanını uygula — tüm fizik hesaplarında bu ölçeklenmiş hız kullanılır
        // Apply velocity gain multiplier — all physics calculations use this scaled velocity
        // gain=1.0 → fizik doğru / gain=0.0 → kompanzasyon kapalı / gain>1.0 → agresif
        double vx = fieldSpeeds.vxMetersPerSecond * velocityGain;
        double vy = fieldSpeeds.vyMetersPerSecond * velocityGain;

        // Robot hızı ihmal edilebilir seviyede ise statik sonuç döndür
        // If robot speed is negligible, return static result
        double robotSpeed = Math.hypot(vx, vy);
        if (robotSpeed < 0.05) {
            double staticDist = turretFieldPos.getDistance(realTarget);
            return new MovingShootResult(realTarget, staticDist, 0, staticDist / shotSpeedMps, 0, 0);
        }

        // =====================================================================
        // ADIM 1: GECİKME TELAFİSİ — Tahmin edilmiş taret pozisyonu
        // STEP 1: LATENCY COMPENSATION — Predicted turret position
        // =====================================================================
        // Top namludan çıktığında robot Δt_latency kadar hareket etmiş olacak.
        // By the time the ball exits the barrel, the robot will have moved Δt_latency.
        Translation2d effectiveTurretPos = turretFieldPos;
        if (enableLatencyComp && mechanicalLatencySec > 0) {
            effectiveTurretPos = new Translation2d(
                turretFieldPos.getX() + vx * mechanicalLatencySec,
                turretFieldPos.getY() + vy * mechanicalLatencySec
            );
        }

        // =====================================================================
        // ADIM 2: 2-İTERASYONLU SANAL HEDEF YAKINSAMASI
        // STEP 2: 2-ITERATION VIRTUAL TARGET CONVERGENCE
        // =====================================================================
        // İterasyon 0: Statik mesafe üzerinden ToF hesapla
        // Iteration 0: Calculate ToF from static distance
        double d0 = effectiveTurretPos.getDistance(realTarget);
        double t0 = d0 / shotSpeedMps;
        Translation2d virtualTarget0 = new Translation2d(
            realTarget.getX() - vx * t0,
            realTarget.getY() - vy * t0
        );

        // İterasyon 1: Güncellenmiş mesafe üzerinden ToF hesapla (yakınsama)
        // Iteration 1: Calculate ToF from updated distance (convergence)
        double d1 = effectiveTurretPos.getDistance(virtualTarget0);
        double t1 = d1 / shotSpeedMps;
        Translation2d virtualTarget = new Translation2d(
            realTarget.getX() - vx * t1,
            realTarget.getY() - vy * t1
        );

        double finalDistance = effectiveTurretPos.getDistance(virtualTarget);
        double finalToF = finalDistance / shotSpeedMps;

        // =====================================================================
        // ADIM 3: RADYAL / TEĞETSEL HIZ AYRIŞTIRMASI
        // STEP 3: RADIAL / TANGENTIAL VELOCITY DECOMPOSITION
        // =====================================================================
        // Birim vektör: taret → sanal hedef
        // Unit vector: turret → virtual target
        Translation2d toTarget = virtualTarget.minus(effectiveTurretPos);
        double dist = toTarget.getNorm();

        double radialVelocity = 0;
        double tangentialVelocity = 0;

        if (dist > 0.01) {
            // Radyal bileşen: hız vektörünün hedefe doğru izdüşümü
            // Radial component: projection of velocity toward target
            // V_radial = (V · û_target) — pozitif = yaklaşma
            // V_radial = (V · û_target) — positive = approaching
            double ux = toTarget.getX() / dist;
            double uy = toTarget.getY() / dist;
            radialVelocity = vx * ux + vy * uy;

            // Teğetsel bileşen: hız vektörünün hedefe dik bileşeni
            // Tangential component: velocity perpendicular to line-of-sight
            // V_tangential = |V × û_target| (2D çarpım z-bileşeni)
            tangentialVelocity = vx * (-uy) + vy * ux;
        }

        // =====================================================================
        // ADIM 4: EFEKTİF MESAFE HESABI
        // STEP 4: EFFECTIVE DISTANCE CALCULATION
        // =====================================================================
        // Robot hedefe yaklaşıyorsa (radialVelocity > 0), top daha kısa yol kat eder → düşük RPM.
        // Robot uzaklaşıyorsa (radialVelocity < 0), top daha uzun yol kat eder → yüksek RPM.
        // If approaching (radialVelocity > 0), ball travels shorter distance → lower RPM.
        // If receding (radialVelocity < 0), ball travels longer distance → higher RPM.
        //
        // effectiveDistance = geometricDistance - V_radial × ToF
        // Bu değer interpolasyon haritasına gönderilir.
        // This value is fed to the interpolation map.
        double effectiveDistance = finalDistance - radialVelocity * finalToF;

        // Güvenlik: Efektif mesafeyi makul sınırlarda tut
        // Safety: Clamp effective distance to reasonable bounds
        effectiveDistance = Math.max(0.3, effectiveDistance);

        // =====================================================================
        // ADIM 5: TARET ÖNCÜLEME AÇISI
        // STEP 5: TURRET LEAD ANGLE
        // =====================================================================
        // Teğetsel hız, hedefin açısal kaymasına neden olur. Taret PID'inin gecikmesini
        // telafi etmek için küçük bir öncüleme açısı ekliyoruz.
        // Tangential velocity causes the target angle to shift. We add a small lead
        // angle to compensate for the turret PID's response time.
        //
        // lead_angle = atan(V_tangential × t_response / distance)
        double turretLeadAngleDeg = 0;
        if (enableTurretLead && dist > 0.3 && turretResponseSec > 0) {
            double leadRad = Math.atan2(tangentialVelocity * turretResponseSec, dist);
            turretLeadAngleDeg = Math.toDegrees(leadRad);

            // Güvenlik: Öncüleme açısını ±10° ile sınırla
            // Safety: Clamp lead angle to ±10°
            turretLeadAngleDeg = Math.max(-10.0, Math.min(10.0, turretLeadAngleDeg));
        }

        return new MovingShootResult(
            virtualTarget,
            effectiveDistance,
            turretLeadAngleDeg,
            finalToF,
            radialVelocity,
            tangentialVelocity
        );
    }

    // =========================================================================
    // LEGACY COMPATIBILITY — Mevcut çağrılar için geriye uyumluluk
    // LEGACY COMPATIBILITY — backward compatible with existing calls
    // =========================================================================

    /**
     * Eski API ile uyumluluk — sadece sanal hedef döndürür.
     * Legacy API compatibility — returns only virtual target.
     *
     * @deprecated Yeni kodda {@link #calculate} kullanın / Use {@link #calculate} in new code
     */
    public static Translation2d getVirtualTarget(
            Translation2d realTarget,
            Translation2d turretFieldPos,
            ChassisSpeeds robotSpeeds,
            double shotSpeedMps,
            double originalDist) {

        MovingShootResult result = calculate(
            realTarget, turretFieldPos, robotSpeeds, shotSpeedMps,
            0.0, 0.0, false, false, 1.0
        );
        return result.virtualTarget();
    }
}
