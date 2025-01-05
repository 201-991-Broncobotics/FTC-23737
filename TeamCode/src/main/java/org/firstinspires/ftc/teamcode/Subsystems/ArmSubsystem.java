package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//TODO: Change this to Autonomous-Only Use
public class ArmSubsystem extends SubsystemBase {

    private final MotorEx extendingMotor;
    private final MotorEx angleMotor;
    private final Telemetry telemetry;
    private final double ticksPerRevolution = 537.7;
    private final double spoolDiameter = 1.1811;
    private final double pulleyDiameter = 3.438;
    private final int extendedBasketDistance = 20;
    private final int extendedGroundDistance = 1;
    private final double angleBasketDistance = 2.7;
    private final double angleGroundDistance = 0;

    public ArmSubsystem(MotorEx extendingMotor, MotorEx angleMotor,
                        Telemetry telemetry) {

        this.telemetry = telemetry;
        this.extendingMotor = extendingMotor;
        this.angleMotor = angleMotor;

    }

    public int inchesToTicks(double inches, double diameter){

        double circumference = Math.PI * diameter;
        double revolutions = inches/circumference;

        return (int) (revolutions * ticksPerRevolution);

    }

    @Override
    public void periodic(){

        telemetry.addData("Extending Motor from Zero: ", extendingMotor.getDistance());
        telemetry.addData("Angle Motor from Zero: ", angleMotor.getDistance());

        if (!extendingMotor.atTargetPosition()){

            extendingMotor.set(1);

        } else if (!angleMotor.atTargetPosition()) {

            angleMotor.set(1);

        } else if (extendingMotor.atTargetPosition()){

            extendingMotor.set(0);

        } else if (angleMotor.atTargetPosition()){

            angleMotor.set(0);

        } else {

            extendingMotor.set(0);
            angleMotor.set(0);

        }
    }

    //CHECKED THIS! GOOD
    public void extendToBasket(){

        extendingMotor.setInverted(false);
        angleMotor.setInverted(true);

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        int extendingTargetPosition = inchesToTicks(extendedBasketDistance, spoolDiameter);
        extendingMotor.setTargetPosition(extendingTargetPosition);
        extendingMotor.setPositionTolerance(14);
        extendingMotor.setVeloCoefficients(0.5, 0, 0.01);

        int angleTargetPosition = inchesToTicks(angleBasketDistance, pulleyDiameter);
        angleMotor.setTargetPosition(angleTargetPosition);
        angleMotor.setPositionTolerance(14);
        angleMotor.setVeloCoefficients(0.5, 0, 0.01);

    }


    public void extendToGround(){

        extendingMotor.setInverted(false);
        angleMotor.setInverted(false);

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        int extendingTargetPosition = inchesToTicks(extendedGroundDistance, spoolDiameter);
        extendingMotor.setTargetPosition(extendingTargetPosition);
        extendingMotor.setPositionTolerance(14);
        extendingMotor.setVeloCoefficients(0.5, 0, 0.01);

        int angleTargetPosition = inchesToTicks(angleGroundDistance, pulleyDiameter);
        angleMotor.setTargetPosition(angleTargetPosition);
        angleMotor.setPositionTolerance(14);
        angleMotor.setVeloCoefficients(0.5, 0, 0.01);

    }

    public void retract(){

        extendingMotor.setInverted(false);
        angleMotor.setInverted(false);

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        extendingMotor.setTargetPosition(0);
        extendingMotor.setPositionTolerance(14);
        extendingMotor.setVeloCoefficients(0.5, 0, 0.01);

        angleMotor.setTargetPosition(0);
        angleMotor.setPositionTolerance(14);
        angleMotor.setVeloCoefficients(0.5, 0, 0.01);

    }

    public void emergencyReset(){

        extendingMotor.setInverted(true);
        angleMotor.setInverted(true);

        extendingMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        int extendingTargetPosition = inchesToTicks(extendedGroundDistance, spoolDiameter);
        extendingMotor.setTargetPosition(extendingTargetPosition);
        extendingMotor.setPositionTolerance(14);
        extendingMotor.setVeloCoefficients(0.5, 0, 0.01);

        int angleTargetPosition = inchesToTicks(angleGroundDistance, pulleyDiameter);
        angleMotor.setTargetPosition(angleTargetPosition);
        angleMotor.setPositionTolerance(14);
        angleMotor.setVeloCoefficients(0.5, 0, 0.01);


    }

    public void resetEncoders(){

        extendingMotor.resetEncoder();
        angleMotor.resetEncoder();

    }

}
