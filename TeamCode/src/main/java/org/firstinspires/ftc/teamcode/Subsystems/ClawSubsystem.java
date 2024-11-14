package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {

    private final ServoEx leftTurnServo;
    private final ServoEx rightTurnServo;
    private final ServoEx leftWheelServo;
    private final ServoEx rightWheelServo;
    private final double specimenTurn = 90;
    private final double sampleDrop = 30;
    private final double sampleRotateAngle = 360;
    private final Telemetry telemetry;

    public ClawSubsystem(ServoEx leftTurnServo, ServoEx rightTurnServo,
                         ServoEx leftWheelServo, ServoEx rightWheelServo,
                         Telemetry telemetry){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.leftWheelServo = leftWheelServo;
        this.rightWheelServo = rightWheelServo;
        this.telemetry = telemetry;

        leftTurnServo.setRange(0, 360, AngleUnit.DEGREES);
        rightTurnServo.setRange(0, 360, AngleUnit.DEGREES);
        leftWheelServo.setRange(0, 360, AngleUnit.DEGREES);
        rightWheelServo.setRange(0, 360, AngleUnit.DEGREES);

    }

    @Override
    public void periodic(){

        telemetry.addData("Left Turn Servo Degrees from Zero: ", leftTurnServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Right Turn Servo Degrees from Zero: ", rightTurnServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Left Wheel Servo Degrees from Zero: ", leftWheelServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Right Wheel Servo Degrees from Zero: ", rightWheelServo.getAngle(AngleUnit.DEGREES));

    }

    public void attachSpecimen(){

        resetInverted();
        reset();

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        leftWheelServo.setInverted(true);
        rightWheelServo.setInverted(false);

        leftTurnServo.rotateByAngle(specimenTurn, AngleUnit.DEGREES);
        rightTurnServo.rotateByAngle(specimenTurn, AngleUnit.DEGREES);

        if (leftTurnServo.getAngle(AngleUnit.DEGREES) == specimenTurn &&
                rightTurnServo.getAngle(AngleUnit.DEGREES) == specimenTurn) {

            leftWheelServo.rotateByAngle(specimenTurn);
            rightWheelServo.rotateByAngle(specimenTurn);

        }
    }

    public void dropSpecimen() {

        resetInverted();
        reset();

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        if (leftTurnServo.getAngle(AngleUnit.DEGREES) == specimenTurn &&
                rightTurnServo.getAngle(AngleUnit.DEGREES) == specimenTurn) {

            leftTurnServo.setInverted(true);
            rightTurnServo.setInverted(false);

            leftTurnServo.rotateByAngle(specimenTurn);
            rightTurnServo.rotateByAngle(specimenTurn);


        } else {

            leftTurnServo.rotateByAngle(specimenTurn, AngleUnit.DEGREES);
            rightTurnServo.rotateByAngle(specimenTurn, AngleUnit.DEGREES);

        }
    }

    public void collectSample(){

        resetInverted();
        reset();

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        leftWheelServo.setInverted(true);
        rightWheelServo.setInverted(false);

        if (leftTurnServo.getAngle(AngleUnit.DEGREES) == sampleDrop &&
                rightTurnServo.getAngle() == sampleDrop){

            leftWheelServo.rotateByAngle(sampleRotateAngle);
            rightWheelServo.rotateByAngle(sampleRotateAngle);

        } else {

            leftTurnServo.rotateByAngle(sampleDrop);
            rightTurnServo.rotateByAngle(sampleDrop);

        }

    }


    public void reset() {

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        leftTurnServo.turnToAngle(0, AngleUnit.DEGREES);
        rightTurnServo.turnToAngle(0, AngleUnit.DEGREES);
        leftWheelServo.turnToAngle(0, AngleUnit.DEGREES);
        rightWheelServo.turnToAngle(0, AngleUnit.DEGREES);

    }

    public void resetInverted(){

        if (leftTurnServo.getInverted()){

            leftTurnServo.turnToAngle(0, AngleUnit.DEGREES);

        } else if (rightTurnServo.getInverted()) {

            rightTurnServo.turnToAngle(0, AngleUnit.DEGREES);

        } else if (leftWheelServo.getInverted()){

            leftWheelServo.turnToAngle(0, AngleUnit.DEGREES);

        } else if (rightWheelServo.getInverted()){

            rightWheelServo.turnToAngle(0, AngleUnit.DEGREES);
        }
    }

}
