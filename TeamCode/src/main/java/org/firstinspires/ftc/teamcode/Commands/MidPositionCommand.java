package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MidPositionCommand extends CommandBase {

    Arm armSubsystem;

    public MidPositionCommand(Arm armSubsystem){

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.specimenLowPosition();

    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
