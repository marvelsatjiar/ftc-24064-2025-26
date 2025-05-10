package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.path.SpecPaths;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

public class Specimen5plus0Pedro extends AbstractAutoPedro {
    private final Follower f = robot.drivetrain;


    @Override
    protected Pose getStartPose() {
        return null;
    }

    @Override
    protected void onRun() {
        scoreFirstSpecimen();
    }


    private void scoreFirstSpecimen() {
        SpecPaths.firstSpec.getPath(0).setPathEndTValueConstraint(0.96);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(f, "start_first_specimen", true),
                        new ParallelAction(
                                new Actions.CallbackAction(RobotActions.setupSpecimen(), SpecPaths.firstSpec, 0, 0),
//                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(0.92)), SpecPaths.firstSpec, 0.8, 0),
                                new FollowPathAction(f, SpecPaths.firstSpec)
                        ),
                        new DrivePoseLoggingAction(f, "setup_first_specimen"),
                        RobotActions.scoreSpecimen()
                )
        );

        robot.actionScheduler.runBlocking();
    }
}
