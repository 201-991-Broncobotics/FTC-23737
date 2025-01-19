package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewClaw extends SubsystemBase {

    private CRServo leftTurnServo, rightTurnServo, pinchServo;
    private Telemetry telemetry;
    private GamepadEx operator;
    public boolean goingUp, goingDown, freeMotion;


    public NewClaw(CRServo leftTurnServo, CRServo rightTurnServo, CRServo pinchServo,
                   Telemetry telemetry, GamepadEx operator){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.pinchServo = pinchServo;
        this.telemetry = telemetry;
        this.operator = operator;

        rightTurnServo.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void ClawFunctionalityCommand(){

        if (goingUp){

            leftTurnServo.setPower(1);
            rightTurnServo.setPower(1);


        }
    }
}
