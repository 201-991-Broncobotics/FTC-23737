package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.HCArm;
import org.firstinspires.ftc.teamcode.Subsystems.HCClaw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "OLD DON'T USE")
public class Comp1 extends CommandOpMode {

    private GamepadEx driver, operator;
    private HCArm arm;
    private HCClaw claw;
    private MecanumSubsystem mecanumSubsystem;

    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        mecanumSubsystem = new MecanumSubsystem(
                new MecanumDrive(
                        new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312)),

                new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312),
                telemetry,
                driver
        );

        arm = new HCArm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                operator
        );

        claw = new HCClaw(
                hardwareMap.get(CRServo.class, "lts"),
                hardwareMap.get(CRServo.class, "rts"),
                hardwareMap.get(CRServo.class, "lws"),
                hardwareMap.get(CRServo.class, "rws"),
                telemetry,
                operator
        );

        register(mecanumSubsystem);
        register(arm);
        register(claw);

        Trigger lower = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger retract = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);

        Trigger clawMotion = new Trigger(() -> operator.getLeftY() != 0.05 || operator.getRightX() != 0.05);


        waitForStart();

        mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));

        lower.whenActive(new InstantCommand(arm::lower, arm));
        lower.whenInactive(new InstantCommand(arm::stopRaising, arm));

        retract.whenActive(new InstantCommand(arm::retract, arm));
        retract.whenInactive(new InstantCommand(arm::stopExtending, arm));

        clawMotion.whenActive(new InstantCommand(() ->
                claw.controlClaw(operator.getLeftY(), operator.getRightX()),
                claw));
        clawMotion.whenInactive(new InstantCommand(claw::stopTurnServos, claw));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(arm::raise, arm));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenInactive(new InstantCommand(arm::stopRaising, arm));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new InstantCommand(arm::extend, arm));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenInactive(new InstantCommand(arm::stopExtending, arm));

        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(arm::resetEncoders, arm));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(arm::resetExtension));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenInactive(new InstantCommand(arm::stopExtending));


        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(claw::collect, claw));
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenInactive(new InstantCommand(claw::stopWheelServos, claw));

        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(claw::drop, claw));
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenInactive(new InstantCommand(claw::stopWheelServos, claw));

        schedule(new RunCommand(telemetry::update));
    }
}
