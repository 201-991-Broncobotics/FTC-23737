package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {

    private ServoEx leftTurnServo, rightTurnServo,
                    leftWheelServo, rightWheelServo;
    private Telemetry telemetry;

    public Claw(ServoEx leftTurnServo, ServoEx rightTurnServo,
                ServoEx leftWheeLServo, ServoEx rightWheelServo,
                Telemetry telemetry){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.leftWheelServo = leftWheeLServo;
        this.rightWheelServo = rightWheelServo;
        this.telemetry = telemetry;

        leftTurnServo.setRange(0, 360, AngleUnit.DEGREES);
        rightTurnServo.setRange(0, 360, AngleUnit.DEGREES);
        leftWheeLServo.setRange(0, 360, AngleUnit.DEGREES);
        rightWheelServo.setRange(0, 360, AngleUnit.DEGREES);

        leftWheeLServo.setInverted(true);
        rightWheelServo.setInverted(false);

    }

    @Override
    public void periodic(){

        telemetry.addData("Left Turn Servo Degrees from Zero: ", leftTurnServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Right Turn Servo Degrees from Zero: ", rightTurnServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Left Wheel Servo Degrees from Zero: ", leftWheelServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Right Wheel Servo Degrees from Zero: ", rightWheelServo.getAngle(AngleUnit.DEGREES));

    }

    public void collectingPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        leftTurnServo.turnToAngle(45, AngleUnit.DEGREES);
        rightTurnServo.turnToAngle(45, AngleUnit.DEGREES);

    }

    public void basketPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        leftTurnServo.turnToAngle(120, AngleUnit.DEGREES);
        rightTurnServo.turnToAngle(120, AngleUnit.DEGREES);

    }

    public void specimenPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        leftTurnServo.turnToAngle(90, AngleUnit.DEGREES);
        rightTurnServo.turnToAngle(90, AngleUnit.DEGREES);

    }

    public void collect(){

        leftWheelServo.setInverted(false);
        rightWheelServo.setInverted(true);

        leftWheelServo.turnToAngle(45, AngleUnit.DEGREES);
        rightWheelServo.turnToAngle(45, AngleUnit.DEGREES);

    }

    public void drop(){

        leftWheelServo.setInverted(true);
        rightWheelServo.setInverted(false);

        leftWheelServo.turnToAngle(45, AngleUnit.DEGREES);
        rightWheelServo.turnToAngle(45, AngleUnit.DEGREES);

    }

}
