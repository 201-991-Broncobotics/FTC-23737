package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "TeleOp - Single Player (WIP)")
public class TeleOpSinglePlayer extends CommandOpMode {

    private GamepadEx driver, emergencyOp;
    private Arm armSubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private Claw clawSubsystem;

    @Override
    public void initialize() {


        driver = new GamepadEx(gamepad1);
        emergencyOp = new GamepadEx(gamepad2);

        mecanumSubsystem = new MecanumSubsystem(
                new MecanumDrive(
                        new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435),
                        new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435),
                        new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435),
                        new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435)),

                new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435),
                telemetry,
                driver
        );

        armSubsystem = new Arm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "em2", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                driver
        );

        clawSubsystem = new Claw(
                new SimpleServo(hardwareMap, "lts", 0, 360),
                new SimpleServo(hardwareMap, "rts", 0, 360),
                new SimpleServo(hardwareMap, "ps", 0, 360),
                driver,
                telemetry);

        register(armSubsystem);
        register(mecanumSubsystem);
        register(clawSubsystem);

        Trigger reset = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger intakeToggle = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);
        Trigger rawExtend = new Trigger(() -> Math.abs(driver.getRightY()) > 0.05);

        armSubsystem.resetEncoders();

        telemetry.addLine("Remember to reset the arm's encoders at the start of TeleOp by pressing A...");
        telemetry.update();

        waitForStart();

        mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));
        armSubsystem.setDefaultCommand(new ArmFunctionalityCommand(armSubsystem));

        reset.whenActive(new InstantCommand(armSubsystem::reset, armSubsystem));
        reset.toggleWhenActive(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(new InstantCommand(clawSubsystem::basketPosition));

        intakeToggle.toggleWhenActive(new InstantCommand(clawSubsystem::grabToggle, clawSubsystem));

        rawExtend.whenActive(new InstantCommand(armSubsystem::rawExtend, armSubsystem));
        rawExtend.whenInactive(new InstantCommand(armSubsystem::setCurrentPosition, armSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::setClawToggle, clawSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(armSubsystem::specimenHighPosition, armSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenHighPosition, clawSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::collect),
                        new InstantCommand(armSubsystem::autonRawRaise),
                        new WaitCommand(60),
                        new InstantCommand(clawSubsystem::drop),
                        new WaitCommand(350),
                        new InstantCommand(armSubsystem::specimenLowPosition)

                ));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(armSubsystem::specimenLoadPosition, armSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(clawSubsystem::specimenLoadPosition, clawSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(armSubsystem::specimenLowPosition, armSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenPosition, clawSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(armSubsystem::rawRaise));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenInactive(new InstantCommand(armSubsystem::setCurrentPosition));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(armSubsystem::rawRaise));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenInactive(new InstantCommand(armSubsystem::setCurrentPosition));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(armSubsystem::rawRaise));

        driver.getGamepadButton(GamepadKeys.Button.B) // Hang
                .whenInactive(new InstantCommand(armSubsystem::setCurrentPosition));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(armSubsystem::resetEncoders, armSubsystem));

        schedule(new RunCommand(telemetry::update));


    }
}
