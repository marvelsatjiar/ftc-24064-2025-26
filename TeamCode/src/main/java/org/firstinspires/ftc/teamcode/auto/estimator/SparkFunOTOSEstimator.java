package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.filter.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;

public class SparkFunOTOSEstimator implements Estimator {
    public static class Params {
        public double localizerTrust = 0.5;

        // See https://github.com/sparkfun/SparkFun_Qwiic_OTOS_FTC_Java_Library/blob/ce6648bc00db6a03fdda073eec79dab91c5e4f4a/SensorSparkFunOTOS.java
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        public double linearScalar = 1.0;
        public double angularScalar = 1.0;
    }

    public static Params PARAMS = new Params();
    private final ComplementaryFilter filter = new ComplementaryFilter(new ComplementaryGains(PARAMS.localizerTrust));

    private final SparkFunOTOS otos;

    private short
            counter = 0,
            counterTarget = 5;


    public SparkFunOTOSEstimator(SparkFunOTOS otos) {
        this.otos = otos;

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(PARAMS.offset);
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.calibrateImu();
        otos.resetTracking();

        FlightRecorder.write("SPARK_FUN_OTOS_PARAMS", PARAMS);
    }

    @Override
    public Pose2d estimate(Pose2d localizerPose) {
        Pose2d pose = localizerPose;

        if (++counter >= counterTarget) {
            SparkFunOTOS.Pose2D otosPose = otos.getPosition();

            if (otosPose != null) {
                pose = new Pose2d(
                        filter.calculate(localizerPose.position.x, otosPose.x),
                        filter.calculate(localizerPose.position.y, otosPose.y),
                        filter.calculate(localizerPose.heading.toDouble(), otosPose.h)
                );
            }
        } else counter = 0;

        return pose;
    }
}
