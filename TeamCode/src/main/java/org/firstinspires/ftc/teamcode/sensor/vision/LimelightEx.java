package org.firstinspires.ftc.teamcode.sensor.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.net.InetAddress;
import java.util.List;

public final class LimelightEx {
    private LLResult result;
    private final Limelight3A limelight;
    private final LED stagelite;

    public LimelightEx(Limelight3A limelight, HardwareMap hardwareMap) {
        this.limelight = limelight;
        this.stagelite = hardwareMap.get(LED.class, "stagelite");
        stagelite.enable(false);
    }

    public LLResult update() {
        result = limelight.getLatestResult();
        return result;
    }

    public LLResult getResult() {
        return result;
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        return result.getColorResults();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        if (result != null) return result.getDetectorResults();
        return null;
    }

    public Pose2d getPoseEstimate() {
        Pose2d pose = null;
        if (result != null && result.isValid()) {
            Pose3D pose3D = result.getBotpose_MT2();
            if (pose3D != null) {
                pose = new Pose2d(pose3D.getPosition().x, pose3D.getPosition().y, 0);
            }
        }
        return pose;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void enableStagelite(boolean enabled){
        stagelite.enable(!enabled);
    }
}