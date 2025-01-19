package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends CommandOpMode {

    private GamepadEx driver, operator;
    private Arm armSubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private Claw clawSubsystem;

    @Override
    public void initialize() {


        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

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
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                operator
        );

        clawSubsystem = new Claw(
                new SimpleServo(hardwareMap, "lts", 0, 360),
                new SimpleServo(hardwareMap, "rts", 0, 360),
                hardwareMap.get(CRServo.class, "ps"),
                telemetry);

        register(armSubsystem);
        register(mecanumSubsystem);
        register(clawSubsystem);

        Trigger basketPosition = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger collect = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);

        armSubsystem.resetEncoders();

        telemetry.addLine("Remember to reset the arm's encoders at the start of TeleOp by pressing A...");
        telemetry.update();

        waitForStart();

        mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));
        armSubsystem.setDefaultCommand(new ArmFunctionalityCommand(armSubsystem));

        basketPosition.whenActive(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
        basketPosition.toggleWhenActive(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(armSubsystem::reset, armSubsystem));

        collect.whenActive(new InstantCommand(armSubsystem::collectPosition, armSubsystem));
        collect.whenActive(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(armSubsystem::drop, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::drop));

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(armSubsystem::rawExtend, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(armSubsystem::rawRaise, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenInactive(new InstantCommand(armSubsystem::setCurrentPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(armSubsystem::specimenHighPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenHighPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(armSubsystem::specimenLowPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::collect, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::drop, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(armSubsystem::resetEncoders, armSubsystem));

        schedule(new RunCommand(telemetry::update));


    }
}
