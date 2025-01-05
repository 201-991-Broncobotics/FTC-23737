package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmFunctionalityCommand extends RunCommand {

    public ArmFunctionalityCommand(Arm armSubsystem){
        super(armSubsystem::armMechanism, armSubsystem);

    }
}
