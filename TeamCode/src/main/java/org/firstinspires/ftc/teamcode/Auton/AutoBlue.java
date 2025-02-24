package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.onbotjava.OnBotJavaStandardFileManager;
import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Commands.AutonomousPathCommandBlue;
import org.firstinspires.ftc.teamcode.Commands.AutonomousPathCommandRed;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "AutoBlue")
public class AutoBlue extends CommandOpMode {

    private Arm armSubsystem;
    private Claw clawSubsystem;
    private GamepadEx operator;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(139, 32.5, Math.toRadians(90));  // Starting position
    private final Pose scorePose = new Pose(126, 18, Math.toRadians(135)); // Scoring position

    private final Pose pickup1Pose = new Pose(120.5, 19, Math.toRadians(158)); // First sample pickup
    private final Pose pickup2Pose = new Pose(121, 19, Math.toRadians(184)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(126, 20, Math.toRadians(210)); // Third sample pickup

    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                if (!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                    follower.followPath(scorePreload, true);
                    if (pathTimer.getElapsedTimeSeconds() > 1){
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));
                    }
                    if (armSubsystem.extendingMotor.getDistance() > 2650 && pathTimer.getElapsedTimeSeconds() > 3.75){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect));
                        if (pathTimer.getElapsedTimeSeconds() > 4.1){
                            setPathState(1);
                        }
                    }
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::autonExtend, armSubsystem));
                    if (pathTimer.getElapsedTimeSeconds() > 2){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4){
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::autonRawRaise));
                        if (pathTimer.getElapsedTimeSeconds() > 5){
                            CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 5.75){
                            CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::specimenLowPosition));
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 7.25) {
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) { // Ensure it only moves when properly in this state
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                    follower.followPath(scorePickup1, true);
                    if (pathTimer.getElapsedTimeSeconds() > 1){
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));
                    }
                    if (armSubsystem.extendingMotor.getDistance() > 2650 && pathTimer.getElapsedTimeSeconds() > 3.75){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.75){
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::autonExtend, armSubsystem));
                    if (pathTimer.getElapsedTimeSeconds() > 3){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5){
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::autonRawRaise));
                        if (pathTimer.getElapsedTimeSeconds() > 5.5){
                            CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 6.5){
                            CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::specimenLowPosition));
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 7.25) {
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()){
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                    follower.followPath(scorePickup2, true);
                    if (pathTimer.getElapsedTimeSeconds() > 1){
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));
                    }
                    if (armSubsystem.extendingMotor.getDistance() > 2650 && pathTimer.getElapsedTimeSeconds() > 3.75){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.75){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition));
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                        CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::reset));
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 7.5){
                        setPathState(-1);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(clawSubsystem::collect, clawSubsystem),
                                    new WaitCommand(2000),
                                    new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem),
                                    new InstantCommand(armSubsystem::autonRawRaise, armSubsystem),
                                    new InstantCommand(armSubsystem::autonRawExtend, armSubsystem)
                            )
                    );
                    setPathState(-1);
                }
                break;
            /*case 4: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::reset, armSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop, clawSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));
                    CommandScheduler.getInstance().schedule(new WaitCommand(1000));
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::collectPosition, armSubsystem));
                    if (armSubsystem.angleMotor.atTargetPosition()) {
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect, clawSubsystem));
                        CommandScheduler.getInstance().schedule(new WaitCommand(250));
                    }
                    setPathState(5);
                }
                break;
*/
            /*case 5: // Wait until the robot is near the second sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));
                    if (armSubsystem.extendingMotor.atTargetPosition()){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop, clawSubsystem));
                    }
                    setPathState(6);
                }
                break;
             */
            case 6: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::reset, armSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop, clawSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::verticalPosition, clawSubsystem));
                    CommandScheduler.getInstance().schedule(new WaitCommand(1000));
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::collectPosition, armSubsystem));
                    if (armSubsystem.angleMotor.atTargetPosition()) {
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect, clawSubsystem));
                        CommandScheduler.getInstance().schedule(new WaitCommand(250));
                    }
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot is near the third sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));
                    if (armSubsystem.extendingMotor.atTargetPosition()){
                        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop, clawSubsystem));
                    }
                    setPathState(8);
                }
                break;

            case 8: // Wait until the robot returns to the scoring position
                CommandScheduler.getInstance().schedule(new InstantCommand(armSubsystem::reset, armSubsystem));
                CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));
                CommandScheduler.getInstance().schedule(new WaitCommand(3000));
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void initialize() {

        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        operator = new GamepadEx(gamepad2);

        armSubsystem = new Arm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "em2", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                operator
        );


        clawSubsystem = new Claw(
                new SimpleServo(hardwareMap, "lts", 0, 360),
                new SimpleServo(hardwareMap, "rts", 0, 360),
                new SimpleServo(hardwareMap, "ps", 0, 360),
                operator,
                telemetry);

        register(armSubsystem, clawSubsystem);

        armSubsystem.resetEncoders();

        waitForStart();


        armSubsystem.setDefaultCommand(new ArmFunctionalityCommand(armSubsystem));
        schedule(new RunCommand(follower::update));
        schedule(new RunCommand(() -> {
            telemetry.addData("Path State", pathState);
            telemetry.addData("Robot Position", follower.getPose().toString());
            telemetry.update();
        }, armSubsystem, clawSubsystem));
        CommandScheduler.getInstance().schedule(new AutonomousPathCommandBlue(this));

    }


}
