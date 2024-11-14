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
import org.firstinspires.ftc.teamcode.Subsystems.HCArm;
import org.firstinspires.ftc.teamcode.Subsystems.HCClaw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "Teleop23737")
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

        Trigger raiseArm = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger lowerArm = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);
        Trigger verticalMotion = new Trigger(() -> operator.getLeftY() != 0);
        Trigger horizontalMotion = new Trigger(() -> operator.getRightX() != 0);


        waitForStart();

        raiseArm.whenActive(new InstantCommand(arm::raise));
        lowerArm.whenActive(new InstantCommand(arm::lower));
        verticalMotion.whenActive(new InstantCommand(claw::verticalMotion));
        horizontalMotion.whenActive(new InstantCommand(claw::verticalMotion));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(arm::extend));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(arm::retract));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(claw::collect));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(claw::drop));

        schedule(new RunCommand(telemetry::update));
    }
}
