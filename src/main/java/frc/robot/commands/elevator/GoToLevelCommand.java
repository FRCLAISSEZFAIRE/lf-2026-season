package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Elevator'ı belirtilen level'a götürür.
 * Hedef pozisyona ulaşınca biter.
 */
public class GoToLevelCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final int targetLevel;

    /**
     * @param elevatorSubsystem Elevator alt sistemi
     * @param level             Hedef level (0=Home, 1=L1, 2=L2, 3=L3, 7=Intake)
     */
    public GoToLevelCommand(ElevatorSubsystem elevatorSubsystem, int level) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetLevel = level;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.goToLevel(targetLevel);
    }

    @Override
    public void execute() {
        // Motion Magic otomatik çalışıyor, ek işlem gerekmez
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevatorSubsystem.stop();
        }
    }
}
