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

import org.firstinspires.ftc.teamcode.Commands.ArmFunctionalityCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
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
                new MotorEx(hardwareMap, "em", 751.8, 223),
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

        register(armSubsystem);
        register(mecanumSubsystem);
        register(clawSubsystem);

        Trigger reset = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0);
        Trigger intakeToggle = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0);
        Trigger rawExtend = new Trigger(() -> Math.abs(operator.getRightY()) > 0.05);
        Trigger rawRaise = new Trigger(() -> Math.abs(operator.getLeftY()) > 0.05);

        armSubsystem.resetEncoders();

        telemetry.addLine("Remember to reset the arm's encoders at the start of TeleOp by pressing A...");
        telemetry.update();

        waitForStart();

        mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));
        armSubsystem.setDefaultCommand(new ArmFunctionalityCommand(armSubsystem));

        reset.whenActive(new InstantCommand(armSubsystem::reset, armSubsystem));
        reset.toggleWhenActive(new InstantCommand(clawSubsystem::collectingPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(armSubsystem::basketPosition, armSubsystem));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(new InstantCommand(clawSubsystem::basketPosition));

        intakeToggle.toggleWhenActive(new InstantCommand(clawSubsystem::grabToggle, clawSubsystem));

        rawExtend.whenActive(new InstantCommand(armSubsystem::rawExtend, armSubsystem));
        rawExtend.whenInactive(new InstantCommand(armSubsystem::setCurrentPosition, armSubsystem));

        rawRaise.whenActive(new InstantCommand(armSubsystem::rawRaise, armSubsystem));
        rawRaise.whenInactive(new InstantCommand(armSubsystem::setCurrentPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::setClawToggle, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(armSubsystem::specimenHighPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenHighPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new IntakeCommand(armSubsystem, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(armSubsystem::specimenLoadPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(clawSubsystem::specimenLoadPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(armSubsystem::specimenLowPosition, armSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .toggleWhenPressed(new InstantCommand(clawSubsystem::specimenPosition, clawSubsystem));

        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(armSubsystem::resetEncoders, armSubsystem));

        schedule(new RunCommand(telemetry::update));


    }
}
