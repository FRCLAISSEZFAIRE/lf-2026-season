package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Elevator'ı joystick ile manuel kontrol eder.
 * Pozitif = yukarı, negatif = aşağı.
 */
public class ManualElevatorCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final DoubleSupplier speedSupplier;

    /**
     * @param elevatorSubsystem Elevator alt sistemi
     * @param speedSupplier     Hız supplier'ı (-1 ile 1 arası)
     */
    public ManualElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier speedSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();

        // Deadband uygula
        if (Math.abs(speed) < 0.1) {
            elevatorSubsystem.stop();
            return;
        }

        // Hızı ölçeklendir
        double velocity = speed * ElevatorConstants.kManualUpVelocity;
        elevatorSubsystem.setVoltage(velocity / ElevatorConstants.kManualUpVelocity * 8.0);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Joystick bırakılana kadar çalışır
    }
}
