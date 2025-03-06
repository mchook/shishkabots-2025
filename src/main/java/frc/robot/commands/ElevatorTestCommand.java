package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTestCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int targetLevel;

    /**
     * Creates a command to test moving the elevator to a specific level
     * @param elevator The elevator subsystem
     * @param level Target level (1, 2, or 3)
     */
    public ElevatorTestCommand(ElevatorSubsystem elevator, int level) {
        this.elevator = elevator;
        this.targetLevel = level;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.goToLevel(targetLevel);
    }

    @Override
    public void execute() {
        // Optional: Add monitoring or additional behavior here
        // For example:
        // SmartDashboard.putNumber("Current Position", elevator.getCurrentPosition());
        // SmartDashboard.putBoolean("Moving Up", elevator.isMovingUp());
    }

    @Override
    public boolean isFinished() {
        return elevator.atTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("it stopped");
            elevator.stop();
        }
    }
}
