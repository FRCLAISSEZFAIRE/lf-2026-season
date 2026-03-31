package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class IntakeIOSim implements IntakeIO {
    private double extensionPositionRevs = 0.0;
    private double rollerVelocityRPM = 0.0;

    private double extensionAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        extensionPositionRevs += extensionAppliedVolts * 0.5 * 0.02;
        rollerVelocityRPM += rollerAppliedVolts * 500.0 * 0.02;
        rollerVelocityRPM *= 0.95;

        inputs.extensionPositionRevs = extensionPositionRevs;
        inputs.extensionAppliedVolts = extensionAppliedVolts;

        inputs.rollerVelocityRPM = rollerVelocityRPM;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
    }

    @Override
    public void setExtensionPositionRevs(double revs) {
        double error = revs - extensionPositionRevs;
        extensionAppliedVolts = MathUtil.clamp(error * 0.5, -12.0, 12.0);
    }

    @Override
    public void setRollerVelocity(double rpm) {
        rollerAppliedVolts = (rpm - rollerVelocityRPM) * 0.1;
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerAppliedVolts = volts;
    }

    @Override
    public void stopRoller() {
        rollerAppliedVolts = 0;
    }

    @Override
    public void stopExtension() {
        extensionAppliedVolts = 0;
    }
}
