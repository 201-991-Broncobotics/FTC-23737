package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "TeleOp TESTING")
public class TeleOpTesting extends CommandOpMode {

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

        armSubsystem = new Arm(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry,
                operator
        );

        clawSubsystem = new Claw(
                hardwareMap.get(ServoEx.class, "lts"),
                hardwareMap.get(ServoEx.class, "rts"),
                hardwareMap.get(ServoEx.class, "lws"),
                hardwareMap.get(ServoEx.class, "rws"),
                telemetry
        );

        register(armSubsystem);
        register(mecanumSubsystem);
        register(clawSubsystem);

        Trigger basketPosition = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger collect = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);

        telemetry.addLine("Remember to Reset the arm's encoders at the start of TeleOp...");
        telemetry.update();

        waitForStart();

        mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));
        armSubsystem.setDefaultCommand(new ArmFunctionalityCommand(armSubsystem));

        basketPosition.whenActive(new InstantCommand(armSubsystem::basketPosition, armSubsystem));

        collect.whenActive(new InstantCommand(armSubsystem::collectPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(armSubsystem::reset, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(armSubsystem::basketPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(armSubsystem::rawExtend, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(armSubsystem::rawRaise, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(clawSubsystem::collect, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(clawSubsystem::basketPosition, clawSubsystem))
                .whenInactive(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(clawSubsystem::drop, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(armSubsystem::resetEncoders, armSubsystem));

        schedule(new RunCommand(telemetry::update));

    }
}
