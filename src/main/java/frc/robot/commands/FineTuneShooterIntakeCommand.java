package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;

/**
 * Command to fine-tune the shooter intake when the coral doesn't go in fully.
 * This runs the motors at a slower speed until the command is canceled.
 */
public class FineTuneShooterIntakeCommand extends Command {
    private final ShooterSubsystem shooter;
    DoubleSupplier shootDoubleSupplier;

    public FineTuneShooterIntakeCommand(ShooterSubsystem shooter, DoubleSupplier shooterInputSupplier) {
        this.shooter = shooter;
        shootDoubleSupplier = shooterInputSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stop();
    }

    @Override
    public void execute() {
        // Just keep the motors running at the fine-tuning speed
        Logger.log("FineTuneShooterIntakeCommand initialized");
        shooter.fineTuneIntake(shootDoubleSupplier.getAsDouble());
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
