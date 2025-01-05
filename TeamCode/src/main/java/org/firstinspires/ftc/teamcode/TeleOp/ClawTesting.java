package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.CRClaw;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

@TeleOp(name = "Claw Testing Stuff")
public class ClawTesting extends CommandOpMode {


    @Override
    public void initialize() {

        GamepadEx operator = new GamepadEx(gamepad2);



        waitForStart();


        schedule(new RunCommand(telemetry::update));

    }


}
