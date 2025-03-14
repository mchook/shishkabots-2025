package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to prepare the shooter for coral intake.
 * This starts the motors at intake velocity and waits for a coral to be detected.
 */
public class PrepareShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    public PrepareShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("PrepareShooterCommand initialized");
        shooter.prepareForIntake();
    }

    @Override
    public void execute() {
        // State management is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("PrepareShooterCommand interrupted");
            shooter.emergencyStop();
        } else {
            System.out.println("PrepareShooterCommand completed normally");
            // No need to stop motors here as the subsystem handles this automatically
        }
    }

    @Override
    public boolean isFinished() {
        // Command is finished when the shooter has a coral inside
        return shooter.getState() == ShooterSubsystem.ShooterState.CORAL_INSIDE;
    }
}
