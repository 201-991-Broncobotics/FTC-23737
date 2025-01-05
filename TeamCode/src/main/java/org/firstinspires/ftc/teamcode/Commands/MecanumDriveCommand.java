package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

public class MecanumDriveCommand extends RunCommand {

    public MecanumDriveCommand(MecanumSubsystem mecanumSubsystem){
        super(mecanumSubsystem::drive, mecanumSubsystem);

    }
}
