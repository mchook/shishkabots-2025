package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;

/**
 * Command to fine-tune the shooter intake when the coral doesn't go in fully.
 * This runs the motors at a slower speed until the command is canceled.
 */
public class FineTuneShooterIntakeCommand extends Command {
    private final ShooterSubsystem shooter;

    public FineTuneShooterIntakeCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Logger.log("FineTuneShooterIntakeCommand initialized");
        shooter.fineTuneIntake();
    }

    @Override
    public void execute() {
        // Just keep the motors running at the fine-tuning speed
    }

    @Override
    public void end(boolean interrupted) {
        Logger.log("FineTuneShooterIntakeCommand ended, interrupted: " + interrupted);
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        // This command runs until canceled
        return false;
    }
}
