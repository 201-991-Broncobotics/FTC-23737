package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

@Autonomous(name = "Get Basket and Park (Not Finished)")
public class BasketAndPark extends CommandOpMode {

    private MecanumDrive driveTrain;
    private Arm armSubsystem;
    private Claw clawSubsystem;
    private GamepadEx operator;

    @Override
    public void initialize() {

        operator = new GamepadEx(gamepad2);

        driveTrain = new MecanumDrive(
                new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435));

        armSubsystem = new Arm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
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

        SequentialCommandGroup autonomousCommandGroup = new SequentialCommandGroup(

                new InstantCommand(() -> driveTrain.driveRobotCentric(0.5, -0.05, -0.125)),
                new InstantCommand(() -> clawSubsystem.collect()),
                new WaitCommand(850),
                new InstantCommand(() -> driveTrain.stop()),
                new WaitCommand(50),
                new InstantCommand(() -> driveTrain.driveRobotCentric(0, 0.05, 0)),
                new WaitCommand(50),
                new InstantCommand(() -> armSubsystem.basketPosition()),
                new InstantCommand(() -> clawSubsystem.basketPosition()),
                new WaitCommand(3000),
                new InstantCommand(() -> clawSubsystem.drop()),
                new WaitCommand(2000),
                new InstantCommand(() -> clawSubsystem.collectingPosition()),
                new InstantCommand(() -> clawSubsystem.collect()),
                new WaitCommand(1000),
                new InstantCommand(() -> armSubsystem.reset()),
                new WaitCommand(3000),
                new InstantCommand(() -> driveTrain.driveRobotCentric(0, -0.5, 0)),
                new WaitCommand(850),
                new InstantCommand(() -> driveTrain.driveRobotCentric(0.5, 0, 0)),
                new WaitCommand(850),
                new InstantCommand(() -> driveTrain.driveRobotCentric(0, 0.5, 0)),
                new WaitCommand(1000
                ));

        schedule(autonomousCommandGroup);


    }
}
