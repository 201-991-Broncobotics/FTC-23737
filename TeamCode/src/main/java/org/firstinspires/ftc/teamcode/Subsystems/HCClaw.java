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

        controlClaw(operator.getLeftY(), operator.getRightX());


    }

    public void controlClaw(double verticalInput, double horizontalInput) {

        if (verticalInput > 0.1 || verticalInput < -0.1) { // Vertical motion

            leftTurnServo.setPower(verticalInput);
            rightTurnServo.setPower(-verticalInput);
            
        } else if (horizontalInput > 0.1 || horizontalInput < -0.1) { // Horizontal/rotational motion


            leftTurnServo.setPower(horizontalInput);
            rightTurnServo.setPower(horizontalInput); // Same direction for rotation

        } else {

            stopTurnServos();

        }
    }

    public void stopTurnServos(){

        leftTurnServo.setPower(0);
        rightTurnServo.setPower(0);

    }

    public void collect(){

        leftWheelServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightWheelServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheelServo.setPower(1);
        rightWheelServo.setPower(1);

    }

    public void drop(){

        leftWheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftWheelServo.setPower(1);
        rightWheelServo.setPower(1);

    }

    public void stopWheelServos(){

        leftWheelServo.setPower(0);
        rightWheelServo.setPower(0);

    }
}
