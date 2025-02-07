package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Testing Thing 2")
public class AutoWhat extends OpMode{
   /* PathBuilder builder = new PathBuilder();

    builder
            .addPath(
            // Line 1
            new BezierLine(
          new Point(9.000, 111.000, Point.CARTESIAN),
          new Point(14.000, 129.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
            .setReversed(true)
      .addPath(
            // Line 2
        new BezierLine(
                    new Point(14.000, 129.000, Point.CARTESIAN),
          new Point(24.000, 120.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
            // Line 3
        new BezierLine(
                    new Point(24.000, 120.000, Point.CARTESIAN),
          new Point(14.000, 129.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
            .addPath(
            // Line 4
        new BezierLine(
                    new Point(14.000, 129.000, Point.CARTESIAN),
          new Point(24.000, 132.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
            // Line 5
        new BezierLine(
                    new Point(24.000, 132.000, Point.CARTESIAN),
          new Point(14.000, 129.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
            .addPath(
            // Line 6
        new BezierLine(
                    new Point(14.000, 129.000, Point.CARTESIAN),
          new Point(45.000, 129.391, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
            .addPath(
            // Line 7
        new BezierLine(
                    new Point(45.000, 129.391, Point.CARTESIAN),
          new Point(14.000, 129.000, Point.CARTESIAN)
        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315));
}
*/



    private final Pose startPose = new Pose(9, 111, Math.toRadians(315));  // Starting position
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315)); // Scoring position

    private final Pose pickup1Pose = new Pose(24, 120, Math.toRadians(0)); // First sample pickup
    private final Pose pickup2Pose = new Pose(24, 132, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(45, 130, Math.toRadians(0)); // Third sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;

    public void buildPaths() {
        // Path for scoring preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path chains for picking up and scoring samples
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Curved path for parking
        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4: // Wait until the robot is near the second sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;

            case 8: // Wait until the robot is near the parking position
                if (!follower.isBusy()) {
                    setPathState(-1); // End the autonomous routine
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

}
