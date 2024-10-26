package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class Arm extends SubsystemBase {

    public DcMotor leftRaisingArmMotor;
    public DcMotor rightRaisingArmMotor;
    public DcMotor leftExtendingArmMotor;
    public DcMotor rightExtendingArmMotor;

    public Arm(HardwareMap map) {


        leftRaisingArmMotor = map.get(DcMotor.class, "lra");
        leftRaisingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRaisingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRaisingArmMotor = map.get(DcMotor.class, "rra");
        rightRaisingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRaisingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftExtendingArmMotor = map.get(DcMotor.class, "lea");
        leftExtendingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftExtendingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightExtendingArmMotor = map.get(DcMotor.class, "rea");
        rightExtendingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightExtendingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void resetEncoders(){

        leftRaisingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRaisingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtendingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtendingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void leftRaise(double lr){

        leftRaisingArmMotor.setPower(lr);

    }

    public void rightRaise(double rr) {

        rightRaisingArmMotor.setPower(rr);

    }

    public void bothRaise(double dr){

            leftRaisingArmMotor.setPower(dr);
            rightRaisingArmMotor.setPower(dr);

    }

    public void leftExtend(double le) {

        leftExtendingArmMotor.setPower(le);

    }

    public void rightExtend(double re) {

        rightExtendingArmMotor.setPower(re);

    }

    public void bothExtend(double be){

            leftExtendingArmMotor.setPower(be);
            rightExtendingArmMotor.setPower(be);

        }
    }
