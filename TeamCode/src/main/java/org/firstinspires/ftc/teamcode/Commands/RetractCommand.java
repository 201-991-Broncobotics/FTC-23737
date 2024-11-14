package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class RetractCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public RetractCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.retract();

    }

    @Override
    public boolean isFinished(){

        return true;

    }
}
