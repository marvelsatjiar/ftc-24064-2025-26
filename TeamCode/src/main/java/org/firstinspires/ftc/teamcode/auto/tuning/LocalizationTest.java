package org.firstinspires.ftc.teamcode.auto.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Drawing;
import org.firstinspires.ftc.teamcode.auto.estimator.CreateAprilTagEstimator;
import org.firstinspires.ftc.teamcode.auto.estimator.CreateIMUEstimator;
import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.drivetrain.TankDrive;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

public final class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.estimator instanceof CreateAprilTagEstimator) {
                ((CreateAprilTagEstimator) drive.estimator).createAprilTagSensor();
            }

            if (drive.estimator instanceof CreateIMUEstimator) {
                ((CreateIMUEstimator) drive.estimator).startIMUThread(this);
            }

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("Heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.estimator instanceof CreateAprilTagEstimator) {
                ((CreateAprilTagEstimator) drive.estimator).createAprilTagSensor();
            }

            if (drive.estimator instanceof CreateIMUEstimator) {
                ((CreateIMUEstimator) drive.estimator).startIMUThread(this);
            }

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("Heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
