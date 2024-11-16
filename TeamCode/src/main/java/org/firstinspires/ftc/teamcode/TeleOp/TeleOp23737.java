package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Commands.DropCommand;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.ResetCommand;
import org.firstinspires.ftc.teamcode.Commands.BasketCommand;
import org.firstinspires.ftc.teamcode.Commands.GroundCommand;
import org.firstinspires.ftc.teamcode.Commands.RetractCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HCClaw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@TeleOp(name = "DON'T USE")
public class TeleOp23737 extends CommandOpMode {

    private GamepadEx driver, operator;
    private ArmSubsystem arm;
    private MecanumSubsystem mecanumSubsystem;
    private ClawSubsystem claw;

    @Override
    public void initialize(){

//Initialize Motors and Subsystems
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

        arm = new ArmSubsystem(
                new MotorEx(hardwareMap, "em", 537.7, 312),
                new MotorEx(hardwareMap, "am", 537.7, 312),
                telemetry
        );


        claw = new ClawSubsystem(
                hardwareMap.get(ServoEx.class, "lts"),
                hardwareMap.get(ServoEx.class, "rts"),
                hardwareMap.get(ServoEx.class, "lws"),
                hardwareMap.get(ServoEx.class, "rws"),
                telemetry
        );

//Register subsystems to add them to the scheduler
        register(arm);
        register(mecanumSubsystem);
        register(claw);

        waitForStart();

        arm.resetEncoders();


            mecanumSubsystem.setDefaultCommand(new MecanumDriveCommand(mecanumSubsystem));

            operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new BasketCommand(arm));
            operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new GroundCommand(arm));
            operator.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new RetractCommand(arm));
            operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                    .whenPressed(new ResetCommand(arm));

            schedule(new RunCommand(telemetry::update));

    }
}
