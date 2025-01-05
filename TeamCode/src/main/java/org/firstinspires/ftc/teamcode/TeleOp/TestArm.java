package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//Simple teleop test to see if the arms functioned if given power. It works, so I think it is just a problem with the position control stuff
@TeleOp(name = "Test Arm")
public class TestArm extends LinearOpMode {

    DcMotorEx em;
    DcMotorEx am;
    GamepadEx driver;
    GamepadEx operator;


    @Override
    public void runOpMode() throws InterruptedException {

        em = hardwareMap.get(DcMotorEx.class, "em");
        am = hardwareMap.get(DcMotorEx.class, "am");

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (operator.isDown(GamepadKeys.Button.A)) {
                am.setPower(1);
            } else am.setPower(0);

        } if (operator.isDown(GamepadKeys.Button.Y)) {

            em.setPower(1);
        } else em.setPower(0);

        if (operator.isDown(GamepadKeys.Button.LEFT_BUMPER)){

            am.setPower(-1);

        } else am.setPower(0);

        if (operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)){

            em.setPower(-1);

        } else em.setPower(0);


    }
}
