package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {

    private ServoEx leftTurnServo, rightTurnServo, pinchServo;
    private Telemetry telemetry;
    private GamepadEx operator;
    private double turnTargetAngle = 0;
    private double pinchTargetAngle = 0;
    public int toggle = 0;
    public int grabToggle = 0;


    public Claw(ServoEx leftTurnServo, ServoEx rightTurnServo,
                ServoEx pinchServo, GamepadEx operator,
                Telemetry telemetry){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.pinchServo = pinchServo;
        this.telemetry = telemetry;
        this.operator = operator;

        leftTurnServo.setRange(-360, 360, AngleUnit.DEGREES);
        rightTurnServo.setRange(-360, 360, AngleUnit.DEGREES);

        pinchServo.setInverted(true);
        rightTurnServo.setInverted(true);

        toggle = 0;
        grabToggle = 0;

    }

    @Override
    public void periodic(){

        telemetry.addData("Left Turn Servo Degrees: ", (leftTurnServo.getPosition() * 360));
        telemetry.addData("Right Turn Servo Degrees: ", (rightTurnServo.getPosition() * 360));
        telemetry.addData("Pinch Servo Angle: ", pinchServo.getAngle());


    }

    public void setClawToggle(){

        if (operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)){

            if (toggle == 0){

                drop();

            } else if (toggle == 1){

                collect();


            }
        }
    }

    public void collect(){

        pinchTargetAngle = 0;

        pinchServo.setPosition(0);

        toggle = 0;

    }

    public void drop(){

        pinchTargetAngle = 0.25;

        pinchServo.setPosition(0.25);

        toggle = 1;

    }

    public void collectingPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        turnTargetAngle = 0;

        leftTurnServo.setPosition(0.9);
        rightTurnServo.setPosition(0.9);

        grabToggle = 0;

    }

    public void basketPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);
        turnTargetAngle = 360;

        leftTurnServo.setPosition(0.2);
        rightTurnServo.setPosition(0.2);

    }

    public void specimenPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        turnTargetAngle = 90;

        leftTurnServo.setPosition(0.45);
        rightTurnServo.setPosition(0.45);

    }

    public void specimenHighPosition(){

        leftTurnServo.setInverted(true);
        rightTurnServo.setInverted(false);

        leftTurnServo.setPosition(0.45);
        rightTurnServo.setPosition(0.45);

    }

    public void verticalPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        rightTurnServo.setPosition(0.9);
        leftTurnServo.setPosition(-0.9);

        grabToggle = 1;

    }

    public void grabToggle(){

        if (Math.abs(operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) != 0) {

            if (grabToggle == 0) {

                verticalPosition();

            } else if (grabToggle == 1) {

                collectingPosition();

            }
        }

    }


}
