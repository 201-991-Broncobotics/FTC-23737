package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;  // Import ElapsedTime

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class IntakeCommand extends CommandBase {

    private final Arm armSubsystem;
    private final Claw clawSubsystem;
    private final ElapsedTime timer = new ElapsedTime();  //

    private int step = 0;

    public IntakeCommand(Arm armSubsystem, Claw clawSubsystem) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        step = 0;
        clawSubsystem.collectingPosition();
        timer.reset();  // Start the timer
    }

    @Override
    public void execute() {
        switch (step) {
            case 0:
                if (timer.seconds() >= 2.0) {  // Wait 2 seconds
                    clawSubsystem.drop();
                    armSubsystem.dropAndPickUp();
                    timer.reset();  // Restart timer for next step
                    step++;
                }
                break;

            case 1:
                if (timer.seconds() >= 1.0) {  // Wait 1 second
                    clawSubsystem.collect();
                    timer.reset();
                    step++;
                }
                break;

            case 2:
                if (timer.seconds() >= 1.0) {  // Wait another 1 second
                    armSubsystem.reset();
                    step++;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return step >= 3;
    }
}
