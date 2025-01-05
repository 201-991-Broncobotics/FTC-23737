package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Hard Coded Arm for Emergency Uses...
public class HCArm extends SubsystemBase {

    private final MotorEx extendingMotor, angleMotor;
    private Telemetry telemetry;
    private GamepadEx operator;
    private int maxTicksMoved;

    public HCArm(MotorEx extendingMotor, MotorEx angleMotor,
                 Telemetry telemetry, GamepadEx operator){

        this.extendingMotor = extendingMotor;
        this.angleMotor = angleMotor;
        this.telemetry = telemetry;
        this.operator = operator;

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        angleMotor.setRunMode(Motor.RunMode.RawPower);
        extendingMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void periodic(){

        telemetry.addData("Extending Motor Power: ", extendingMotor.get());
        telemetry.addData("Extending Motor Ticks: ", extendingMotor.getDistance());
        telemetry.addData("Angle Motor Power: ", angleMotor.get());
        telemetry.addData("Angle Motor Ticks: ", angleMotor.getDistance());

        if (extendingMotor.getDistance() > 3600 || extendingMotor.getDistance() < -3600){

            extendingMotor.stopMotor();
            extendingMotor.set(0);
            telemetry.addLine("Extending Too Much!!!");


        }

        //Not tested yet
        if (angleMotor.getDistance() < 1000 && !tryingToRaise()){

            angleMotor.set(0.1);
            telemetry.addLine("Counteracting Gravity...");

        }
    }

    public int maxTicksMoved(double inches){

        double ticksPerRev = 537.7;
        double spoolDiameter = 1.1811;

        double circumference = Math.PI * spoolDiameter;
        double revolutions = inches/circumference;

        return (int) (revolutions * ticksPerRev);
    }

    public void raise(){

        angleMotor.set(1);

    }

    public void lower(){

        angleMotor.set(-1);

    }

    public void stopRaising(){

        angleMotor.set(0);

    }

    public void extend() {

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        extendingMotor.set(-1);

    }

    public void retract(){

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        extendingMotor.set(1);

    }

    public void resetExtension(){

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);

        extendingMotor.setTargetPosition(10);
        extendingMotor.setPositionTolerance(10);
        extendingMotor.setVeloCoefficients(0.5, 0, 0.01);

        extendingMotor.set(0.5);


    }

    public void stopExtending(){

        extendingMotor.set(0);

    }

    public void yank(){

        angleMotor.set(-1);

        if (maxTicksMoved(3) > 200 || maxTicksMoved(3) < -200){

            angleMotor.set(0);

        }
    }

    public void resetEncoders(){

        extendingMotor.stopAndResetEncoder();
        angleMotor.stopAndResetEncoder();

    }

    //Not implemented yet
    public boolean tryingToRaise(){

        if (operator.isDown(GamepadKeys.Button.LEFT_BUMPER) ||
                operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0){

            return true;

        } else {

            return false;
        }
    }
}
