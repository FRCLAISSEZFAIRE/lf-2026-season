package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
    private final AHRS m_gyro;

    public GyroIONavX() {
        m_gyro = new AHRS(NavXComType.kUSB1);
        m_gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = m_gyro.isConnected();
        // NavX returns CW positive, WPILib expects CCW positive for traditional math (unless Odometry is already handling it, but typically we invert NavX yaw).
        // Let's stick to the raw angle conversion and let DriveSubsystem handle the Math.
        // Wait, standard WPILib is CCW positive. NavX is CW positive.
        // Usually, `getAngle()` returns continuous CW positive.
        // `getRotation2d()` on AHRS actually negates it properly (CCW positive).
        inputs.yawPositionRad = m_gyro.getRotation2d().getRadians();
        // Yaw rate is degrees per second. Needs to be negated to match rotation2d.
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-m_gyro.getRate());

        inputs.pitchPositionRad = Units.degreesToRadians(m_gyro.getPitch());
        inputs.pitchVelocityRadPerSec = 0.0; // NavX doesn't provide direct pitch rate easily

        inputs.rollPositionRad = Units.degreesToRadians(m_gyro.getRoll());
        inputs.rollVelocityRadPerSec = 0.0; // NavX doesn't provide direct roll rate easily
    }

    @Override
    public void reset() {
        m_gyro.reset();
    }

    @Override
    public void setYaw(double yawDegrees) {
        // NavX doesn't have a direct setAngle, but we can reset and use offsets in DriveSubsystem
        m_gyro.reset();
        m_gyro.setAngleAdjustment(yawDegrees);
    }
}
