package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class CollectCommand extends CommandBase {

    Arm armSubsystem;

    public CollectCommand(Arm armSubsystem){

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.collectPosition();

    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
