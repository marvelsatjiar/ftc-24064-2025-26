package org.firstinspires.ftc.teamcode.auto.localizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.message.SparkFunOTOSInputsMessage;

@Config
public final class SparkFunOTOSLocalizer implements Localizer {
    public static class Params {
        // See https://github.com/sparkfun/SparkFun_Qwiic_OTOS_FTC_Java_Library/blob/ce6648bc00db6a03fdda073eec79dab91c5e4f4a/SensorSparkFunOTOS.java
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 1.125, -Math.PI / 2);
        public double linearScalar = 60/60.3954;
        public double angularScalar = 3600/(3600-32.5);
    }

    public static Params PARAMS = new Params();

    public final SparkFunOTOS otos;

    private SparkFunOTOS.Pose2D lastPos;
    private boolean initialized;

    public SparkFunOTOSLocalizer(HardwareMap hardwareMap) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(PARAMS.offset);
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.calibrateImu();
        otos.resetTracking();

        FlightRecorder.write("SPARK_FUN_OTOS_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity();

        FlightRecorder.write("SPARK_FUN_OTOS_INPUTS", new SparkFunOTOSInputsMessage(pos, vel));

        if (!initialized) {
            initialized = true;

            lastPos = pos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        SparkFunOTOS.Pose2D posDelta = new SparkFunOTOS.Pose2D(pos.x - lastPos.x, pos.y - lastPos.y, pos.h - lastPos.h);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {posDelta.x, vel.x}),
                        new DualNum<>(new double[] {posDelta.y, vel.y})
                ),
                new DualNum<>(new double[] {posDelta.h, vel.h})
        );

        lastPos = pos;

        return twist;
    }
}
