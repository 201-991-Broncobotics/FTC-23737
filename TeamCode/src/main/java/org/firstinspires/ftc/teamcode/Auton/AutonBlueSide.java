package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@Autonomous(name = "DO NOT USE")
public class AutonBlueSide extends CommandOpMode {

    private Arm armSubsystem;
    private Claw clawSubsystem;
    private Trajectory straightTrajectory, collectTrajectory;

    @Override
    public void initialize() {

    }
}
