package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class GroundCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public GroundCommand(ArmSubsystem armSubsystem){

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.extendToGround();

    }

    @Override
    public boolean isFinished(){

        return true;

    }

}
