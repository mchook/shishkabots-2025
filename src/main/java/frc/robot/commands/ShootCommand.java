package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Timer timer;
    private static final double SHOOT_DURATION = 3.0; // Run the motor for 3 seconds and stop

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("Starting shooter motors");
        shooter.shoot();
    }

    @Override
    public void execute() {
        // Optional: Add monitoring or additional behavior here
        System.out.println("Shooter running: " + timer.get() + " seconds");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(SHOOT_DURATION);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.stop();
        System.out.println("Shooter stopped after " + timer.get() + " seconds");
    }
}
