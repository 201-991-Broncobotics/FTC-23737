package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Hard Coded Claw for Emergency Use...
public class HCClaw extends SubsystemBase {

    private final CRServo leftTurnServo, rightTurnServo, leftWheelServo, rightWheelServo;
    private Telemetry telemetry;
    private GamepadEx operator;

    public HCClaw(CRServo leftTurnServo, CRServo rightTurnServo,
                  CRServo leftWheelServo, CRServo rightWheelServo,
                  Telemetry telemetry,GamepadEx operator){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.leftWheelServo = leftWheelServo;
        this.rightWheelServo = rightWheelServo;
        this.telemetry = telemetry;
        this.operator = operator;


    }

    @Override
    public void periodic(){

        telemetry.addData("LTS Power: ", leftTurnServo.getPower());
        telemetry.addData("RTS Power: ", rightTurnServo.getPower());
        telemetry.addData("LWS Power: ", leftWheelServo.getPower());
        telemetry.addData("RWS Power: ", rightWheelServo.getPower());

    }

    public void verticalMotion(){

        if (operator.getRightX() > -0.05 ||
            operator.getRightX() < 0.05){

            leftTurnServo.setDirection(DcMotorSimple.Direction.FORWARD);
            rightTurnServo.setDirection(DcMotorSimple.Direction.FORWARD);

            leftTurnServo.setPower(operator.getLeftY());
            rightTurnServo.setPower(operator.getLeftY());

        } else if (operator.getRightX() < -0.05||
        operator.getRightX() > 0.05){

            leftTurnServo.setDirection(DcMotorSimple.Direction.REVERSE);
            rightTurnServo.setDirection(DcMotorSimple.Direction.FORWARD);

            leftTurnServo.setPower(operator.getRightX());
            rightTurnServo.setPower(operator.getRightX());

        }
    }

    public void collect(){

        leftWheelServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightWheelServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheelServo.setPower(0.33);
        rightWheelServo.setPower(0.33);

    }

    public void drop(){

        leftWheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftWheelServo.setPower(0.33);
        rightWheelServo.setPower(0.33);

    }
}
