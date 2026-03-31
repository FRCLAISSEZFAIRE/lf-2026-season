package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    
    private final VisionIO leftIO;
    private final VisionIO rightIO;
    private final VisionIOInputsAutoLogged leftInputs = new VisionIOInputsAutoLogged();
    private final VisionIOInputsAutoLogged rightInputs = new VisionIOInputsAutoLogged();

    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
    private boolean visionEnabled = true;

    private boolean leftHasValidPose = false;
    private boolean rightHasValidPose = false;
    private boolean hasValidPose = false;
    private int totalTagCount = 0;
    private Pose2d latestVisionPose = new Pose2d();
    private double latestTimestamp = 0.0;

    public VisionSubsystem(VisionIO leftIO, VisionIO rightIO, DriveSubsystem drive) {
        this.leftIO = leftIO;
        this.rightIO = rightIO;
        this.drive = drive;

        tuningTable.getEntry("Vision/Enabled").setDefaultBoolean(true);

        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.kLimelightLeft,
                VisionConstants.kLeftCameraToRobot.getTranslation().getX(),
                VisionConstants.kLeftCameraToRobot.getTranslation().getY(),
                VisionConstants.kLeftCameraToRobot.getTranslation().getZ(),
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getX()),
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getY()),
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getZ())
        );
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.kLimelightRight,
                VisionConstants.kRightCameraToRobot.getTranslation().getX(),
                VisionConstants.kRightCameraToRobot.getTranslation().getY(),
                VisionConstants.kRightCameraToRobot.getTranslation().getZ(),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getX()),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getY()),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getZ())
        );

        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightLeft, VisionConstants.kAprilTagPipelineIndex);
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightRight, VisionConstants.kAprilTagPipelineIndex);
    }

    @Override
    public void periodic() {
        visionEnabled = tuningTable.getEntry("Vision/Enabled").getBoolean(true);
        Logger.recordOutput("Tuning/Vision/Enabled", visionEnabled);

        double gyroYawDegrees = drive.getRotation2d().getDegrees();
        double gyroRateDegPerSec = Units.radiansToDegrees(drive.getGyroVelocityRadPerSec());

        leftIO.setRobotOrientation(gyroYawDegrees, gyroRateDegPerSec);
        rightIO.setRobotOrientation(gyroYawDegrees, gyroRateDegPerSec);

        leftIO.updateInputs(leftInputs);
        rightIO.updateInputs(rightInputs);
        Logger.processInputs("Vision/Left", leftInputs);
        Logger.processInputs("Vision/Right", rightInputs);

        if (!visionEnabled) {
            hasValidPose = false; leftHasValidPose = false; rightHasValidPose = false; totalTagCount = 0;
            return;
        }

        processCameraPose(leftInputs, gyroRateDegPerSec, true);
        processCameraPose(rightInputs, gyroRateDegPerSec, false);

        totalTagCount = leftInputs.tagCount + rightInputs.tagCount;
        hasValidPose = leftHasValidPose || rightHasValidPose;

        Logger.recordOutput("Tuning/Vision/HasValidPose", hasValidPose);
        Logger.recordOutput("Tuning/Vision/TotalTagCount", totalTagCount);
        Logger.recordOutput("Tuning/Vision/LeftCamera/Valid", leftHasValidPose);
        Logger.recordOutput("Tuning/Vision/RightCamera/Valid", rightHasValidPose);
    }

    private void processCameraPose(VisionIOInputsAutoLogged inputs, double gyroRateDegPerSec, boolean isLeft) {
        if (isLeft) leftHasValidPose = false;
        else rightHasValidPose = false;

        String name = isLeft ? "Left" : "Right";

        if (!inputs.hasData) {
            Logger.recordOutput("Tuning/Vision/" + name + "/RejectReason", "NoData");
            return;
        }

        if (Math.abs(gyroRateDegPerSec) > VisionConstants.kMaxGyroRateForVision) {
            Logger.recordOutput("Tuning/Vision/" + name + "/RejectReason", "HighGyroRate");
            return;
        }

        if (inputs.pose.getX() < -0.5 || inputs.pose.getX() > 17.0 ||
            inputs.pose.getY() < -0.5 || inputs.pose.getY() > 9.0) {
            Logger.recordOutput("Tuning/Vision/" + name + "/RejectReason", "OutOfField");
            return;
        }

        double xyStdev = calculateXYStdDev(inputs, gyroRateDegPerSec);
        double thetaStdev = 9999999.0;

        drive.addVisionMeasurement(inputs.pose, inputs.timestampSeconds, VecBuilder.fill(xyStdev, xyStdev, thetaStdev));

        if (isLeft) leftHasValidPose = true;
        else rightHasValidPose = true;

        latestVisionPose = inputs.pose;
        latestTimestamp = inputs.timestampSeconds;

        Logger.recordOutput("Tuning/Vision/" + name + "/RejectReason", "None");
        Logger.recordOutput("Tuning/Vision/" + name + "/XYStdDev", xyStdev);
    }

    private double calculateXYStdDev(VisionIOInputsAutoLogged inputs, double gyroRate) {
        double stdev = 0.5;
        if (Math.abs(gyroRate) > 100.0) stdev += 0.5;
        if (inputs.tagCount > 1) stdev -= 0.3;
        if (inputs.avgTagDist > VisionConstants.kMaxReliableTagDistance) stdev += 0.5;
        else if (inputs.avgTagDist > VisionConstants.kHighUncertaintyTagDistance) stdev += 0.2;
        return Math.max(0.1, stdev);
    }

    public boolean isVisionEnabled() { return visionEnabled; }
    public boolean hasValidPoseEstimate() { return hasValidPose; }
    public int getTotalTagCount() { return totalTagCount; }
    public Pose2d getLatestVisionPose() { return latestVisionPose; }
    public double getLatestTimestamp() { return latestTimestamp; }
    public int getTagCount() { return totalTagCount; }
}