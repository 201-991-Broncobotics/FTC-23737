package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class DropBasketCommand extends CommandBase {

    private Arm armSubsystem;
    private Claw clawSubsystem;

    public DropBasketCommand(Arm armSubsystem, Claw clawSubsystem){

        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;

        addRequirements(armSubsystem, clawSubsystem);

    }

    @Override
    public void initialize(){

        armSubsystem.basketPosition();

    }

    @Override
    public void execute(){

        clawSubsystem.collect();
    }
}
