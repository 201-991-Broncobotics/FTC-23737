package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

import java.util.concurrent.TimeUnit;

public class IntakeCommand extends CommandBase {

    private final Arm armSubsystem;
    private final Claw clawSubsystem;
    private final Timing.Timer timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
    private final Timing.Timer timer2 = new Timing.Timer(250, TimeUnit.MILLISECONDS);
    private final Timing.Timer timer3 = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    private int step = 0;
    private boolean commandScheduled = false;  // ✅ Prevents repeat scheduling

    public IntakeCommand(Arm armSubsystem, Claw clawSubsystem) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collectingPosition));
        step = 0;
        commandScheduled = false;
    }

    @Override
    public void execute() {
        switch (step) {
            case 0:
                if (!commandScheduled) {
                    timer.start();
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::drop));
                    CommandScheduler.getInstance().schedule(new CollectCommand(armSubsystem));
                    commandScheduled = true;
                }
                if (timer.done()) {
                    step++;
                    commandScheduled = false;
                }
                break;

            case 1:
                if (!commandScheduled) {
                    timer2.start();
                    CommandScheduler.getInstance().schedule(new InstantCommand(clawSubsystem::collect));
                    commandScheduled = true;
                }
                if (timer2.done()) {
                    step++;
                    commandScheduled = false;
                }
                break;

            case 2:
                if (!commandScheduled) {
                    timer3.start();
                    CommandScheduler.getInstance().schedule(new MidPositionCommand(armSubsystem));
                    commandScheduled = true;
                }
                if (timer3.done()) {
                    step++;
                    commandScheduled = false;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return step >= 3;
    }
}
