package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ResetCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ResetCommand(ArmSubsystem armSubsystem){

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.emergencyReset();


    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
