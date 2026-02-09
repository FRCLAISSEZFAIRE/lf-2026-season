package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;

public final class Configs {
        public static final class MAXSwerveModule {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double nominalVoltage = 12.0;
                        double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(60); // Vortex can handle more, e.g. 60-80A
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.04, 0, 0)
                                        .outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(25); // NEO 550 için güvenli limit (20-25A önerilir)

                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0) // radians per second
                                        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
