package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.control.filter.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

public class LimelightEstimator implements Estimator {
    public static class Params {
        // For example, 0.4 means localizer will be weighted 40% while April Tag estimate will be weighted 60%
        public double localizerTrust = 0.4;
    }

    public static Params PARAMS = new Params();
    private final ComplementaryFilter filter = new ComplementaryFilter(new ComplementaryGains(PARAMS.localizerTrust));
    private LimelightEx limelight;

    public LimelightEstimator(LimelightEx limelight) {
        this.limelight = limelight;
    }

    @Override
    public Pose2d estimate(Pose2d localizerPose) {
        Pose2d tagEstimate = limelight.getPoseEstimate();
        if (tagEstimate == null) {
            return localizerPose;
        }
        return new Pose2d(
                new Vector2d(
                        filter.calculate(localizerPose.position.x, tagEstimate.position.x),
                        filter.calculate(localizerPose.position.y, tagEstimate.position.y)
                ),
                localizerPose.heading
        );
    }
}
