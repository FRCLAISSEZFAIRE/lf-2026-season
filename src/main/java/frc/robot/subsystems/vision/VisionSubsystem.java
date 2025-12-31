package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    /**
     * Eğer geçerli bir görüntü varsa, Pose ve Zaman bilgisini paketleyip döndürür.
     * DriveSubsystem bunu alıp Odometry'sine ekleyecek.
     */
    public Optional<VisionMeasurement> getBestMeasurement() {
        if (inputs.hasTarget && inputs.tagCount > 0) {
            // Basit bir filtre: Eğer tag'e çok uzaksak (örn 4 metre) güvenme diyebiliriz.
            if(inputs.avgTagDist > 4.0) return Optional.empty();

            return Optional.of(new VisionMeasurement(inputs.estimatedPose, inputs.timestamp));
        }
        return Optional.empty();
    }

    // Veri transferi için yardımcı sınıf (Record)
    public record VisionMeasurement(Pose2d pose, double timestamp) {}
}