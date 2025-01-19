package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {

    private ServoEx leftTurnServo, rightTurnServo;
    private CRServo pinchServo;
    private Telemetry telemetry;
    private double turnTargetAngle = 0;
    private double pinchTargetAngle = 0;

    public Claw(ServoEx leftTurnServo, ServoEx rightTurnServo,
                CRServo pinchServo,
                Telemetry telemetry){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.pinchServo = pinchServo;
        this.telemetry = telemetry;

        leftTurnServo.setRange(0, 360, AngleUnit.DEGREES);
        rightTurnServo.setRange(0, 360, AngleUnit.DEGREES);

        pinchServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTurnServo.setInverted(true);

        leftTurnServo.setPosition(0.5);
        rightTurnServo.setPosition(0.5);

    }

    @Override
    public void periodic(){

        telemetry.addData("Left Turn Servo Degrees: ", (leftTurnServo.getPosition() * 360));
        telemetry.addData("Right Turn Servo Degrees: ", (rightTurnServo.getPosition() * 360));
        telemetry.addData("Pinch Servo Power: ", pinchServo.getPower());


    }

    public void collect(){

        pinchTargetAngle = 0;

        pinchServo.setPower(1);

    }

    public void drop(){

        pinchTargetAngle = 0.25;

        pinchServo.setPower(-1);

    }

    public void collectingPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        turnTargetAngle = 0;

        leftTurnServo.setPosition(0);
        rightTurnServo.setPosition(0);

        drop();


    }

    public void basketPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);
        turnTargetAngle = 360;

        leftTurnServo.setPosition(0.9);
        rightTurnServo.setPosition(0.9);

    }

    public void specimenPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        turnTargetAngle = 90;

        leftTurnServo.setPosition(0.5);
        rightTurnServo.setPosition(0.5);

        drop();

    }

    public void specimenHighPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        leftTurnServo.setPosition(0.75);
        rightTurnServo.setPosition(0.75);

        drop();

    }


}
