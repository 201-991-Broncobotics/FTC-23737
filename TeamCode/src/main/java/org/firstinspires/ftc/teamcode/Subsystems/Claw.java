package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class Claw extends SubsystemBase {

    private ServoEx leftTurnServo, rightTurnServo, pinchServo;
    private Telemetry telemetry;
    private GamepadEx operator;
    private double turnTargetAngle = 0;
    private double pinchTargetAngle = 0;
    public int toggle = 0;
    public int grabToggle = 0;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public Claw(ServoEx leftTurnServo, ServoEx rightTurnServo,
                ServoEx pinchServo, GamepadEx operator,
                Telemetry telemetry){

        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.pinchServo = pinchServo;
        this.telemetry = telemetry;
        this.operator = operator;

        leftTurnServo.setRange(0, 180, AngleUnit.DEGREES);
        rightTurnServo.setRange(0, 180, AngleUnit.DEGREES);

        pinchServo.setInverted(false);
        rightTurnServo.setInverted(true);

        toggle = 0;
        grabToggle = 0;

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

        pinchTargetAngle = 0.35;

        pinchServo.setPosition(0.35);
        toggle = 0;

    }

    public void drop(){

        pinchTargetAngle = 1;

        pinchServo.setPosition(1);

        toggle = 1;

    }

    public void collectingPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        turnTargetAngle = 0;

        leftTurnServo.setPosition(0.4);
        rightTurnServo.setPosition(0.4);

        grabToggle = 1;

    }

    public void basketPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);
        turnTargetAngle = 360;

        leftTurnServo.setPosition(1);
        rightTurnServo.setPosition(1);

    }

    public void specimenPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        turnTargetAngle = 90;

        leftTurnServo.setPosition(0.75);
        rightTurnServo.setPosition(0.75);

    }

    public void specimenHighPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        leftTurnServo.setPosition(0.67);
        rightTurnServo.setPosition(0.67);

    }

    public void specimenLoadPosition(){

        timer.reset();

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(true);

        leftTurnServo.setPosition(0.6);
        rightTurnServo.setPosition(0.6);

        if (timer.time(TimeUnit.MILLISECONDS) > 125){

            setClawToggle();


        }

    }

    public void verticalPosition(){

        leftTurnServo.setInverted(false);
        rightTurnServo.setInverted(false);

        rightTurnServo.setPosition(0.74);
        leftTurnServo.setPosition(0.59);

        grabToggle = 0;

    }

    public void grabToggle(){

        if (Math.abs(operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) != 0) {

            if (grabToggle == 0) {

                collectingPosition();

            } else if (grabToggle == 1) {

                verticalPosition();

            }
        }

    }


}
