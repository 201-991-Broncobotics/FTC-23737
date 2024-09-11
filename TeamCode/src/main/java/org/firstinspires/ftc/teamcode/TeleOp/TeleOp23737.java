package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends CommandOpMode {


    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive driveTrain;
    private GamepadEx driver, operator;
    private Translation2d frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation;

    @Override
    public void initialize(){
        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight); //Mecanum :3

        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_1150); //TODO: Change GoBILDA Motors Based on RPMs, these are placeholders
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_1150);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_1150);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_1150);

        frontLeft.stopAndResetEncoder();
        frontRight.stopAndResetEncoder();
        backLeft.stopAndResetEncoder();
        backRight.stopAndResetEncoder();

        frontLeftLocation = new Translation2d(0.25, 0.25); //x and Y need to be relative distances from the center
        frontRightLocation = new Translation2d(0.25, -0.25);
        backLeftLocation = new Translation2d(-0.25, 0.25);
        backRightLocation = new Translation2d(-0.25, -0.25);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            driveTrain.driveRobotCentric(
                    driver.getLeftX(),
                    driver.getLeftY(),
                    driver.getRightY());
        }
    }




}
