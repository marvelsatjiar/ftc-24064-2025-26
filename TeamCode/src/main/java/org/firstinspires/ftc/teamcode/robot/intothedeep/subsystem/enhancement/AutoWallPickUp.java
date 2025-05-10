package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;

@Config
public class AutoWallPickUp {
    private final LimelightEx limelightEx;

    private final PIDController headingPID = new PIDController();
    private final PIDController detectionXDegreesPID = new PIDController();

    public static PIDGains headingGains = new PIDGains(
            1.15,
            0,
            0.0001
    );

    public static PIDGains detectionXDegreesGains = new PIDGains(
            0.0288875,
            0.00007125,
            0.00001486525
    );

    public static double kMovement = 0.0573;

    private State currentDetectionXDegrees = new State(0);
    private State currentHeading = new State(0);


    public AutoWallPickUp(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        this.limelightEx.getLimelight().pipelineSwitch(Common.LIMELIGHT_SPECIMEN_NN_PIPELINE);
        this.limelightEx.getLimelight().setPollRateHz(10);

        headingPID.setGains(headingGains);
        detectionXDegreesPID.setGains(detectionXDegreesGains);

        headingPID.setTarget(new State(Math.toRadians(180)));
        detectionXDegreesPID.setTarget(new State(0));
    }

    public PoseVelocity2d run(double x, double y) {
        limelightEx.update();

        List<LLResultTypes.DetectorResult> results = limelightEx.getDetectorResult();
        LLResultTypes.DetectorResult result;

        headingPID.setGains(headingGains);
        detectionXDegreesPID.setGains(detectionXDegreesGains);

        double theta = robot.drivetrain.headingOffset - robot.drivetrain.pose.heading.toDouble();

        if (theta < 0) theta += Math.PI * 2;

        currentHeading = new State(theta);
        double headingPower = -headingPID.calculate(currentHeading);

        if (headingPower < kMovement && headingPower > 0) headingPower = kMovement;
        else if (headingPower < 0 && headingPower > -kMovement) headingPower = kMovement;

        if (results != null && results.size() >= 1) {
            result = results.get(0);
        } else {
            return new PoseVelocity2d(
                    new Vector2d(
                            x,
                            y
                    ),
                    headingPower
            );
        }

        currentDetectionXDegrees = new State(result.getTargetXDegrees());

        double translationPower = detectionXDegreesPID.calculate(currentDetectionXDegrees);

        if (translationPower < kMovement && translationPower > 0) translationPower = kMovement;
        else if (translationPower < 0 && translationPower > -kMovement) translationPower = kMovement;

        return new PoseVelocity2d(
                new Vector2d(
                        x,
                        translationPower
                ),
                headingPower
        );
    }

    public void printTelemetry() {
        mTelemetry.addData("current heading", Math.toDegrees(currentHeading.x));
        mTelemetry.addData("current detection x degrees", currentDetectionXDegrees.x);
        mTelemetry.addData("target heading", 180);
        mTelemetry.addData("target detection x degrees", 0);
        mTelemetry.addData("all detections", limelightEx.getDetectorResult());
    }
}
