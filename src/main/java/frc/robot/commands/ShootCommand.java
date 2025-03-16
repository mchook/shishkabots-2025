package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to shoot a coral and then lower the elevator.
 * This starts the shooter motors and waits until the coral is ejected,
 * then automatically lowers the elevator to level 1 (intake position).
 */
public class ShootCommand extends SequentialCommandGroup {
    
    /**
     * Creates a command to shoot a coral and then lower the elevator
     * @param shooter The shooter subsystem
     * @param elevator The elevator subsystem
     */
    public ShootCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
        addCommands(
            // First shoot the coral
            new ShootCoralCommand(shooter, elevator)
            
            // Then lower the elevator to level 1 (intake position)
            //new ElevatorTestCommand(elevator, 1)
        );
    }
    
    /**
     * Inner command that handles just the shooting part
     */
    private static class ShootCoralCommand extends Command {
        private final ShooterSubsystem shooter;
        private final ElevatorSubsystem elevator;

        public ShootCoralCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
            this.shooter = shooter;
            this.elevator = elevator;
            addRequirements(shooter);
        }

        @Override
        public void initialize() {
            System.out.println("Starting shooter motors");
            // have to create a elevator level for very bottom in order for this to run
            /* if (elevator.getCurrentLevel() == 0) {
                shooter.shootBottomLevelCoral();
            } */
            shooter.shootCoral();
        }

        @Override
        public void execute() {
            // State management is handled in the subsystem
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                System.out.println("Shooter stopped after interruption");
                shooter.emergencyStop();
            } else {
                System.out.println("Shooter stopped normally");
                // No need to stop motors here as the subsystem handles this automatically
            }
        }

        @Override
        public boolean isFinished() {
            // Command is finished when the shooter returns to NO_CORAL state
            return shooter.getState() == ShooterSubsystem.ShooterState.NO_CORAL;
        }
    }
}
