package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Auton.AutoBlue;

public class AutonomousPathCommandBlue extends CommandBase {
    private final AutoBlue autoBlue;

    public AutonomousPathCommandBlue(AutoBlue autoBlue) {
        this.autoBlue = autoBlue;
    }

    @Override
    public void execute() {
        autoBlue.autonomousPathUpdate();  // Keep updating the path
    }

    @Override
    public boolean isFinished() {
        return false;  //So it runs forever
    }
}
