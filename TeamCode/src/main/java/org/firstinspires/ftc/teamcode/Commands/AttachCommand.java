package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

public class AttachCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public AttachCommand(ClawSubsystem clawSubsystem) {

        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);

    }

    @Override
    public void initialize() {

        clawSubsystem.attachSpecimen();

    }

    @Override
    public boolean isFinished(){

        return true;

    }
}
