package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Armgrabber;

@TeleOp(name = "TeleOp23737")
public class TeleOp23737 extends CommandOpMode {


    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive driveTrain;
    private GamepadEx driver, operator;
    private Arm arm;
    private Armgrabber armgrabber;

    @Override
    public void initialize(){

//Initialize Motors and Subsystems
        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        arm = new Arm(hardwareMap);
        armgrabber = new Armgrabber(hardwareMap);

        frontLeft.stopAndResetEncoder();
        frontRight.stopAndResetEncoder();
        backLeft.stopAndResetEncoder();
        backRight.stopAndResetEncoder();
//Initialize FTC Lib's Mecanum Drivetrain
        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight); //Mecanum :3
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
//Register subsystems to add them to the scheduler
        register(arm);
        register(armgrabber);

//Initialize Trigger conditions
        Trigger leftRaisingArmActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.LEFT_BUMPER));
        Trigger rightRaisingActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger bothRaisingArmsActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.LEFT_BUMPER) && operator.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger leftExtendingArmActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_LEFT));
        Trigger rightExtendingArmActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_RIGHT));
        Trigger bothExtendingArmsActive = new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger armGrab = new Trigger(() -> operator.getLeftY() < 0 || operator.getLeftY() > 0);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.driveRobotCentric(driver.getLeftX(), driver.getLeftY(), driver.getRightX()); //Drivetrain Code
            leftRaisingArmActive.whenActive(new InstantCommand(() -> arm.leftRaise(operator.getLeftY()))); //Arm Mechanisms
            rightRaisingActive.whenActive(new InstantCommand(() -> arm.rightRaise(operator.getLeftY())));
            bothRaisingArmsActive.whenActive(new InstantCommand(() -> arm.bothRaise(operator.getLeftY())));
            leftExtendingArmActive.whenActive(new InstantCommand(() -> arm.leftExtend(operator.getRightY())));
            rightExtendingArmActive.whenActive(new InstantCommand(() -> arm.rightExtend(operator.getRightY())));
            bothExtendingArmsActive.whenActive(new InstantCommand(() -> arm.bothExtend(operator.getRightY())));
            armGrab.whenActive(new InstantCommand(() -> armgrabber.Grab(operator.getRightX()))); //Arm Servo Mechanisms
            armGrab.whenInactive(new InstantCommand(() -> armgrabber.brake()));

        }
    }
}
