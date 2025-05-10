package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagPlusIMUEstimator implements CreateAprilTagEstimator, CreateIMUEstimator {
    private final IMUEstimator imuEstimator;
    private final AprilTagEstimator aprilTagEstimator;

    public AprilTagPlusIMUEstimator(HardwareMap hardwareMap) {
        this.aprilTagEstimator = new AprilTagEstimator(hardwareMap);
        this.imuEstimator = new IMUEstimator(hardwareMap);
    }

    @Override
    public Pose2d estimate(Pose2d localizerPose) {
        return imuEstimator.estimate(aprilTagEstimator.estimate(localizerPose));
    }

    @Override
    public void createAprilTagSensor() {
        aprilTagEstimator.createAprilTagSensor();
    }

    @Override
    public void startIMUThread(LinearOpMode opMode) {
        imuEstimator.startIMUThread(opMode);
    }
}
