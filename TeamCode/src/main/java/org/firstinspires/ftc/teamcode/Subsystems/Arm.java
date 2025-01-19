package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {

    private final MotorEx extendingMotor, angleMotor;
    private Telemetry telemetry;
    private final GamepadEx operator;
    private final double
            ticksPerRevolution = 537.7,
            spoolDiameter = 1.181,
            pulleyDiameter = 3.438,
            positionPower = 0.25;
    private double exponentialPower;
    public int extendingTargetPosition, angleTargetPosition;
    private boolean extendingIsInPosition, angleIsInPosition;

    public Arm(MotorEx extendingMotor, MotorEx angleMotor,
               Telemetry telemetry, GamepadEx operator){

        this.extendingMotor = extendingMotor;
        this.angleMotor = angleMotor;
        this.telemetry = telemetry;
        this.operator = operator;

        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

    }

    public void armMechanism(){

        telemetry.addData("Angle Motor Power: ", angleMotor.get());
        telemetry.addData("Angle Motor Current Position: ", angleMotor.getDistance());
        telemetry.addData("Angle Motor Target Position: ", angleTargetPosition);
        telemetry.addData("Extending Motor Power: ", extendingMotor.get());
        telemetry.addData("Extending Motor Current Position: ", extendingMotor.getDistance());
        telemetry.addData("Extending Motor Target Position: ", extendingTargetPosition);

        if (extendingIsInPosition){

            extendingMotor.setTargetPosition(extendingTargetPosition);

            if (!extendingMotor.atTargetPosition()){

                extendingMotor.set(positionPower);

            } else extendingMotor.set(0);

        } else if (operator.isDown(GamepadKeys.Button.X)) {

            extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            if (operator.getLeftX() < 0){

                exponentialPower = Math.pow(operator.getLeftX(), 2);
                extendingMotor.set(-exponentialPower);
            } else if (operator.getLeftX() > 0){

                exponentialPower = Math.pow(operator.getLeftX(), 2);
                extendingMotor.set(exponentialPower);
            } else {

                extendingMotor.set(0);

            }

        }

        if (angleIsInPosition){

            angleMotor.setTargetPosition(angleTargetPosition);

            if (!angleMotor.atTargetPosition()){

                angleMotor.set(0.02);

            } else angleMotor.set(0);

        } else if (operator.isDown(GamepadKeys.Button.X)){

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            if (operator.getLeftY() < 0){

                exponentialPower = Math.pow(operator.getLeftY(), 2);
                angleMotor.set(-exponentialPower);

            } else if (operator.getLeftY() > 0){

                exponentialPower = Math.pow(operator.getLeftY(), 2);
                angleMotor.set(exponentialPower);
            } else {

                angleMotor.set(0);

            }

        }

        if (Math.abs(extendingMotor.getDistance()) > 3500){

            extendingMotor.set(0);
            telemetry.addLine("Overextending!!!!");

        }
    }

    public void basketPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true; 

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(25);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = 3300;
        angleTargetPosition = 1450;

    }

    public void collectPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true;

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendingMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(25);
        angleMotor.setPositionTolerance(25);

        extendingTargetPosition = 0;
        angleTargetPosition = 200;

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

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(100);
        angleMotor.setPositionTolerance(100);

        extendingTargetPosition = 0;
        angleTargetPosition = 200;

    }

    public void rawExtend(){

        extendingIsInPosition = false;

        extendingMotor.setRunMode(Motor.RunMode.RawPower);

        extendingTargetPosition = 0;

    }

    public void rawRaise(){

        angleIsInPosition = false;

        angleMotor.setRunMode(Motor.RunMode.RawPower);

        angleTargetPosition = 0;

    }

    public void specimenHighPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);

        angleTargetPosition = 1350;
        extendingTargetPosition = 0;

    }

    public void specimenLowPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);

        angleTargetPosition = 575;
        extendingTargetPosition = 0;

    }

    public void setCurrentPosition(){

        angleIsInPosition = true;
        extendingIsInPosition = true;

        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionTolerance(25);
        extendingMotor.setPositionCoefficient(0.5);
        extendingMotor.setPositionTolerance(25);

        angleTargetPosition = (int) angleMotor.getDistance();
        extendingTargetPosition = (int) extendingMotor.getDistance();

    }

    public void resetEncoders(){

        extendingMotor.resetEncoder();
        angleMotor.resetEncoder();
    }
}