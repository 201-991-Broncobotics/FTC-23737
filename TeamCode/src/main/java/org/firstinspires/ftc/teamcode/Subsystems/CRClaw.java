package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CRClaw extends SubsystemBase {

    private final CRServo leftTurnServo, rightTurnServo,
                          pinchServo;
    private Telemetry telemetry;
    public double turnPower;
    public double pinchPower;

    public CRClaw(CRServo leftTurnServo, CRServo rightTurnServo,
                  CRServo pinchServo,
                  Telemetry telemetry) {

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.pinchServo = pinchServo;
        this.telemetry = telemetry;

        pinchServo.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void periodic(){

        telemetry.addData("LTS Power: ", leftTurnServo.getPower());
        telemetry.addData("RTS Power: ", rightTurnServo.getPower());
        telemetry.addData("PS Power: ", pinchServo.getPower());

    }

    public void basketPosition(){

        turnPower = 1;

        leftTurnServo.setPower(-turnPower);
        rightTurnServo.setPower(turnPower);

    }

    public void collectingPosition(){

        turnPower = -1;

        leftTurnServo.setPower(-turnPower);
        rightTurnServo.setPower(turnPower);

    }

    public void drop(){

        pinchPower = 0.25;

        pinchServo.setPower(pinchPower);

    }

    public void close(){

        pinchPower = -0.25;

    }



}
