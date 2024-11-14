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

    public HCArm(MotorEx extendingMotor, MotorEx angleMotor,
                 Telemetry telemetry, GamepadEx operator){

        this.extendingMotor = extendingMotor;
        this.angleMotor = angleMotor;
        this.telemetry = telemetry;
        this.operator = operator;

        extendingMotor.setRunMode(Motor.RunMode.RawPower);
        angleMotor.setRunMode(Motor.RunMode.RawPower);

    }

    @Override
    public void periodic(){

        telemetry.addData("Extending Motor Power: ", extendingMotor.get());
        telemetry.addData("Angle Motor Power: ", angleMotor.get());

    }

    public void raise(){

        angleMotor.set(operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

    }

    public void lower(){

        angleMotor.set(-operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

    }

    public void extend(){

        extendingMotor.set(0.25);

    }

    public void retract(){

        extendingMotor.set(-0.25);

    }
}
