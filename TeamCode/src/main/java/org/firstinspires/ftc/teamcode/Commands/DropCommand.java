package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

public class DropCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public DropCommand(ClawSubsystem clawSubsystem) {

        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);

    }

    @Override
    public void initialize() {

        clawSubsystem.dropSpecimen();

    }

    @Override
    public boolean isFinished() {

        return true;

    }
}
