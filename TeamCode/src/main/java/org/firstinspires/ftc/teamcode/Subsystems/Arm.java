package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {

    public final MotorEx extendingMotor, extendingMotor2, angleMotor;
    private Telemetry telemetry;
    private final GamepadEx operator;
    private final double
            ticksPerRevolution = 537.7,
            spoolDiameter = 1.181,
            pulleyDiameter = 3.438,
            positionPower = 1;
    private double trackedPosition;
    private double anglePower = 0.02;
    private double autonAnglePower = 0;
    private double autonExtensionPower = 0;
    public int extendingTargetPosition, angleTargetPosition;
    private boolean extendingIsInPosition, angleIsInPosition;

    public Arm(MotorEx extendingMotor, MotorEx extendingMotor2, MotorEx angleMotor,
               Telemetry telemetry, GamepadEx operator){

        this.extendingMotor = extendingMotor;
        this.extendingMotor2 = extendingMotor2;
        this.angleMotor = angleMotor;
        this.telemetry = telemetry;
        this.operator = operator;

        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

    }

    public void armMechanism() {

        telemetry.addData("Angle Motor Power: ", angleMotor.get());
        telemetry.addData("Angle Motor Current Position: ", angleMotor.getDistance());
        telemetry.addData("Angle Motor Target Position: ", angleTargetPosition);
        telemetry.addData("Extending Motor Powers: ", extendingMotor.get());
        telemetry.addData("Extending Motor Current Position: ", extendingMotor.getDistance());
        telemetry.addData("Extending Motor Target Position: ", extendingTargetPosition);

        if (extendingIsInPosition) {

            extendingMotor.setTargetPosition(extendingTargetPosition);
            extendingMotor2.setTargetPosition(extendingTargetPosition);

            if (!extendingMotor.atTargetPosition()) {

                extendingMotor.set(positionPower);
                extendingMotor2.set(positionPower);

            } else {
                extendingMotor.set(0);
                extendingMotor2.set(0);
            }

        } else if (Math.abs(operator.getRightY()) > 0.05) {

            extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            extendingMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            extendingMotor.set(-operator.getRightY());
            extendingMotor2.set(-operator.getRightY());


        } else if (Math.abs(autonExtensionPower) > 0){

            extendingMotor.set(autonExtensionPower);
            extendingMotor2.set(autonExtensionPower);

            } else {

            extendingMotor.set(0);
            extendingMotor2.set(0);

        }


        if (angleIsInPosition) {

            angleMotor.setTargetPosition(angleTargetPosition);

            if (!angleMotor.atTargetPosition()) {

                angleMotor.set(anglePower);

            } else angleMotor.set(0);

        } else if (Math.abs(operator.getLeftY()) > 0.05) {

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(operator.getLeftY());

        } else if (operator.isDown(GamepadKeys.Button.Y)){

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(0.4);

        } else if (operator.isDown(GamepadKeys.Button.X)){

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(-0.25);

        } else if (operator.isDown(GamepadKeys.Button.B)) {

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(1);
        } else if (Math.abs(autonAnglePower) > 0) {

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(-0.5);
        } else {

            angleMotor.set(0);

        }



        if (Math.abs(extendingMotor.getDistance()) > 3150){

            extendingMotor.set(0);
            extendingMotor2.set(0);
            telemetry.addLine("Overextending!!!!");

        }
    }

    public void basketPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor2.setPositionCoefficient(0.5);
        angleMotor.setPositionCoefficient(0.2);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionTolerance(25);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = 2750;
        angleTargetPosition = 1800;

    }

    public void collectPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true;
        anglePower = 0.01;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setPositionCoefficient(0.05);
        extendingMotor2.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionTolerance(25);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = (int) extendingMotor.getDistance();
        angleTargetPosition = -200;

    }

    public void drop(){

        angleIsInPosition = false;

        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        angleMotor.setRunMode(Motor.RunMode.RawPower);

        angleTargetPosition = 0;
    }

    public void reset(){

        extendingIsInPosition = true;
        angleIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setPositionCoefficient(0.05);
        extendingMotor2.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionTolerance(25);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = 0;
        angleTargetPosition = 300;

    }

    public void rawExtend(){

        extendingIsInPosition = false;
        angleIsInPosition = true;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        extendingMotor2.setRunMode(Motor.RunMode.RawPower);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        angleMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = 0;
        angleTargetPosition = (int) angleMotor.getDistance();

        if (angleTargetPosition >= 200 && angleTargetPosition <= 500){

            angleTargetPosition = 300;

        } else if (angleTargetPosition >= 1800 && angleTargetPosition <= 2000){

            angleTargetPosition = 1975;

        }


    }

    public void rawRaise(){

        angleIsInPosition = false;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.RawPower);

        angleTargetPosition = 0;

    }

    public void specimenHighPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.5);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionCoefficient(0.5);
        extendingMotor2.setPositionTolerance(25);

        angleTargetPosition = 1250;
        extendingTargetPosition = -75;

    }

    public void specimenLoadPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.5);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionCoefficient(0.5);
        extendingMotor2.setPositionTolerance(25);

        angleTargetPosition = 900;
        extendingTargetPosition = (int) extendingMotor.getDistance();

    }

    public void specimenLowPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.5);
        angleMotor.setPositionTolerance(10);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionCoefficient(0.5);
        extendingMotor2.setPositionTolerance(25);

        angleTargetPosition = 300;
        extendingTargetPosition = 0;

    }

    public void setCurrentPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.25);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.25);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionCoefficient(0.25);
        extendingMotor2.setPositionTolerance(25);

        angleTargetPosition = (int) angleMotor.getDistance();
        extendingTargetPosition = (int) extendingMotor.getDistance();

    }

    public void autonExtend(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.02;
        autonAnglePower = 0;
        autonExtensionPower = 0;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor2.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.5);
        angleMotor.setPositionTolerance(10);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);
        extendingMotor2.setPositionCoefficient(0.5);
        extendingMotor2.setPositionTolerance(25);

        angleTargetPosition = 350;
        extendingTargetPosition = 3000;

    }

    public void dropAndPickUp(){

        angleIsInPosition = true;
        extendingIsInPosition = true;
        anglePower = 0.01;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.5);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);

        angleTargetPosition = 100;
        extendingTargetPosition = (int) extendingMotor.getDistance();

    }

    public void autonRawRaise(){

        angleIsInPosition = false;
        autonAnglePower = -1;

        angleMotor.setRunMode(Motor.RunMode.RawPower);

        angleTargetPosition = 0;

    }

    public void autonRawExtend(){

        extendingIsInPosition = false;
        angleIsInPosition = true;
        autonExtensionPower = 1;

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        extendingMotor2.setRunMode(Motor.RunMode.RawPower);


        extendingTargetPosition = 0;
        angleTargetPosition = 0;


    }

    public void resetEncoders(){

        extendingMotor.resetEncoder();
        angleMotor.resetEncoder();

        extendingTargetPosition = 0;
        angleTargetPosition = 0;
    }
}