package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Elevator encoder pozisyonunu sıfırlar (homing).
 * Anında biter.
 */
public class ResetElevatorCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;

    public ResetElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.resetPosition();
    }

    @Override
    public boolean isFinished() {
        return true; // Anında biter
    }
}
