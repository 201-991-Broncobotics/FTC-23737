package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumSubsystem extends SubsystemBase {

    private final MecanumDrive mecanumDrive;
    private final Motor frontLeft, backLeft, frontRight, backRight;
    private Telemetry telemetry;
    public double strafeSpeed,forwardSpeed, turnSpeed;
    private final GamepadEx driver;

    public MecanumSubsystem(MecanumDrive mecanumDrive,
                            Motor frontLeft, Motor backLeft, Motor frontRight, Motor backRight,
                            Telemetry telemetry, GamepadEx driver){

        this.mecanumDrive = mecanumDrive;
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.telemetry = telemetry;
        this.driver = driver;

        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setInverted(true);
        backRight.setInverted(true);

    }

    public void drive(){

        telemetry.addData("Front Left Power: ", frontLeft.get());
        telemetry.addData("Front Right Power: ", frontRight.get());
        telemetry.addData("Back Left Power: ", backLeft.get());
        telemetry.addData("Back Right Power: ", backRight.get());

        strafeSpeed = driver.getLeftX();
        forwardSpeed = driver.getLeftY();
        turnSpeed = driver.getRightX();

        mecanumDrive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);

    }
}
