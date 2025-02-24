package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Auton.AutoRed;

public class AutonomousPathCommandRed extends CommandBase {
    private final AutoRed autoRed;  // Reference to your AutoRed class

    public AutonomousPathCommandRed(AutoRed autoRed) {
        this.autoRed = autoRed;
    }

    @Override
    public void execute() {
        autoRed.autonomousPathUpdate();  // Keep updating the path
    }

    @Override
    public boolean isFinished() {
        return false;  // Runs forever until the OpMode stops
    }
}
