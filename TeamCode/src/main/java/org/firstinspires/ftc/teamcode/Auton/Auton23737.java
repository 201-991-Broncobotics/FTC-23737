package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.HCArm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "DON'T USE")
public class Auton23737 extends CommandOpMode {

    private SampleMecanumDrive driveTrain;
    private GamepadEx operator;
    private HCArm arm;
    private boolean isRedTeam;
    public Trajectory forwardTrajectory, strafeTrajectory, straightenTrajectory;

    @Override
    public void initialize() {

        //TODO: Set Red Team true or False depending on..if you're red team or not
        isRedTeam = true;

        driveTrain = new SampleMecanumDrive(hardwareMap);

        arm = new HCArm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                operator
        );

        register(arm);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        driveTrain.setPoseEstimate(startPose);

        forwardTrajectory = driveTrain.trajectoryBuilder(startPose)
                .forward(3)
                .build();

        if (isRedTeam) {

             strafeTrajectory = driveTrain.trajectoryBuilder(forwardTrajectory.end())
                    .strafeRight(2)
                    .build();

            straightenTrajectory = driveTrain.trajectoryBuilder(strafeTrajectory.end())
                    .forward(3)
                    .build();

        } else {

            strafeTrajectory = driveTrain.trajectoryBuilder(forwardTrajectory.end())
                    .strafeLeft(2)
                    .build();

            straightenTrajectory = driveTrain.trajectoryBuilder(strafeTrajectory.end())
                    .forward(3)
                    .build();
        }

        SequentialCommandGroup autonomousCommandGroup = new SequentialCommandGroup(
                new InstantCommand(() -> driveTrain.followTrajectory(forwardTrajectory)), // Drive forward
                new WaitCommand(500),  // Wait for half a second
                new InstantCommand(() -> driveTrain.followTrajectory(strafeTrajectory)), // Strafe right
                new WaitCommand(500),  // Wait for half a second
                new InstantCommand(() -> driveTrain.followTrajectory(straightenTrajectory)), // Straighten forward
                new InstantCommand(() -> arm.yank()));

        schedule(autonomousCommandGroup);


    }
}
