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
            power = 0.25;
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

                extendingMotor.set(power);

            } else extendingMotor.set(0);

        } else if (operator.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON))

            extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            extendingMotor.set(-operator.getLeftY());

        if (angleIsInPosition){

            angleMotor.setTargetPosition(angleTargetPosition);

            if (!angleMotor.atTargetPosition()){

                angleMotor.set(0.02);

            } else angleMotor.set(0);

        } else if (operator.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){

            angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            angleMotor.set(operator.getRightY());

        };
    }

    public void basketPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true; 

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(100);
        angleMotor.setPositionTolerance(10);

        extendingTargetPosition = 3600;
        angleTargetPosition = 1600;

    }

    public void collectPosition(){

        extendingIsInPosition = true;
        angleIsInPosition = true;

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);
        extendingMotor.setPositionCoefficient(0.05);
        angleMotor.setPositionCoefficient(0.05);
        extendingMotor.setPositionTolerance(100);
        angleMotor.setPositionTolerance(50);

        extendingTargetPosition = 1500;
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

    public int inchesToTicks(double inches, double diameter){

        double circumference = Math.PI * diameter;
        double revolutions = inches/circumference;

        return (int) (revolutions * ticksPerRevolution);

    }

    public void resetEncoders(){

        extendingMotor.resetEncoder();
        angleMotor.resetEncoder();
    }
}