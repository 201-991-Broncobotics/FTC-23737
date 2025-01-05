package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton23737 CRASH RIGHT")
public class CrashTheWall2 extends CommandOpMode {

    private MecanumDrive driveTrain;

    @Override
    public void initialize() {

        driveTrain = new MecanumDrive(
                new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312));

        waitForStart();

        SequentialCommandGroup autonomousCommandGroup = new SequentialCommandGroup(

                new InstantCommand(() -> driveTrain.driveRobotCentric(-0.5, 0, 0)),
                new WaitCommand(5000),
                new InstantCommand(() -> driveTrain.stop()
                ));

        schedule(autonomousCommandGroup);
    }
}
