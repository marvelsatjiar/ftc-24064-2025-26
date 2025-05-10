package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.path;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class SpecPaths {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain firstSpec = builder
            .addPath(
                    new BezierLine(
                            new Point(8.291, 65.000, Point.CARTESIAN),
                            new Point(39.000, 76.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain pushingIntermediary = builder
            .addPath(
                    new BezierCurve(
                            new Point(39.000, 76.000, Point.CARTESIAN),
                            new Point(30.000, 74.000, Point.CARTESIAN),
                            new Point(17.500, 41.000, Point.CARTESIAN),
                            new Point(52.000, 35.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain sample1 = builder
            .addPath(
                    new BezierCurve(
                            new Point(52.000, 35.000, Point.CARTESIAN),
                            new Point(60.000, 32.500, Point.CARTESIAN),
                            new Point(60.000, 23.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain goToSample2 = builder
            .addPath(
                    new BezierLine(
                            new Point(60.000, 23.000, Point.CARTESIAN),
                            new Point(20.000, 23.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain sample2 = builder
            .addPath(
                    new BezierCurve(
                            new Point(20.000, 23.000, Point.CARTESIAN),
                            new Point(62.000, 28.400, Point.CARTESIAN),
                            new Point(60.000, 13.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain goToSample3 = builder
            .addPath(
                    new BezierLine(
                            new Point(60.000, 13.000, Point.CARTESIAN),
                            new Point(20.000, 12.500, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain sample3 = builder
            .addPath(
                    new BezierCurve(
                            new Point(20.000, 12.500, Point.CARTESIAN),
                            new Point(57.000, 18.000, Point.CARTESIAN),
                            new Point(60.000, 7.500, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain scoreSecondSpec = builder
            .addPath(
                    new BezierLine(
                            new Point(60.000, 7.500, Point.CARTESIAN),
                            new Point(8.300, 7.500, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain grabSpec = builder
            .addPath(
                    new BezierCurve(
                            new Point(8.300, 7.500, Point.CARTESIAN),
                            new Point(22.000, 3.000, Point.CARTESIAN),
                            new Point(15.000, 75.500, Point.CARTESIAN),
                            new Point(39.000, 72.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
            .build();

    public static PathChain scoreSpec = builder
            .addPath(
                    new BezierCurve(
                            new Point(39.000, 72.000, Point.CARTESIAN),
                            new Point(17.250, 63.000, Point.CARTESIAN),
                            new Point(32.000, 51.000, Point.CARTESIAN),
                            new Point(18.000, 24.250, Point.CARTESIAN),
                            new Point(9.500, 27.500, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))
            .build();
}
