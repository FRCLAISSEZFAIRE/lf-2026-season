package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final String name;
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Simulation State
    private double m_simDrivePositionMeters = 0;
    private double m_simTurnPositionRad = 0;
    private double m_simDriveVelocityMetersPerSec = 0;

    /**
     * Constructs a MAXSwerveModule.
     * 
     * @param io                   The IO layer for this module
     * @param name                 The name of the module (e.g., "FrontLeft")
     * @param chassisAngularOffset Angular offset for this module in radians
     */
    public MAXSwerveModule(ModuleIO io, String name, double chassisAngularOffset) {
        this.io = io;
        this.name = name;
        this.m_chassisAngularOffset = chassisAngularOffset;

        // Initialize desired state angle based on current offset
        m_desiredState.angle = new Rotation2d(0);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + name, inputs);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(inputs.turnAbsolutePositionRad));

        // Command driving and turning IO towards their respective setpoints.
        io.setDriveVelocity(correctedDesiredState.speedMetersPerSecond);
        io.setTurnPosition(correctedDesiredState.angle.getRadians());

        // Store the state actually sent to motors (optimized) for simulation
        m_desiredState = correctedDesiredState;
    }

    public void updatePID(double driveP, double turnP) {
        io.updatePID(driveP, turnP);
    }

    public void updateInversions(boolean driveInverted, boolean turnInverted) {
        io.updateInversions(driveInverted, turnInverted);
    }

    public void resetEncoders() {
        io.resetEncoders();

        // Sim reset
        m_simDrivePositionMeters = 0;
        m_simTurnPositionRad = 0;
    }

    public void simulationPeriodic(double dt) {
        // Hızlanma simülasyonu (Kraken/NEO torku için bir yakınsama faktörü)
        // Hedef hıza anında ulaşmak yerine her döngüde %40'ını kapatır
        double driveConvergence = 0.4; 
        double targetDriveVel = -m_desiredState.speedMetersPerSecond;
        m_simDriveVelocityMetersPerSec += (targetDriveVel - m_simDriveVelocityMetersPerSec) * driveConvergence;
        
        m_simDrivePositionMeters += m_simDriveVelocityMetersPerSec * dt;

        // Dönüş simülasyonu (Modülün dönme hızı kısıtlaması)
        // Modül saniyede ~20 radyan dönebilir gibi simüle edilir
        double turnConvergence = 0.5;
        double targetTurnPos = m_desiredState.angle.getRadians();
        m_simTurnPositionRad += (targetTurnPos - m_simTurnPositionRad) * turnConvergence;
    }

    public SwerveModulePosition getPosition() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation() && !(io instanceof ModuleIOSpark)) {
            return new SwerveModulePosition(
                    m_simDrivePositionMeters,
                    new Rotation2d(m_simTurnPositionRad - m_chassisAngularOffset));
        }

        // Apply chassis angular offset to the encoder position to get the position relative to the chassis.
        // Negate drive encoder: MAXSwerve bevel gear reverses shaft vs wheel direction.
        return new SwerveModulePosition(
                -inputs.drivePositionMeters,
                new Rotation2d(inputs.turnAbsolutePositionRad - m_chassisAngularOffset));
    }

    public SwerveModuleState getState() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation() && !(io instanceof ModuleIOSpark)) {
            return new SwerveModuleState(
                    m_simDriveVelocityMetersPerSec,
                    new Rotation2d(m_simTurnPositionRad - m_chassisAngularOffset));
        }

        return new SwerveModuleState(
                -inputs.driveVelocityMetersPerSec,
                new Rotation2d(inputs.turnAbsolutePositionRad - m_chassisAngularOffset));
    }
}
