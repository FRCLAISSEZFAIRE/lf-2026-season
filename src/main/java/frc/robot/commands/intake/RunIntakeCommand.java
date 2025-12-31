package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Intake motorunu verilen voltajla çalıştırır.
 */
public class RunIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final double voltage;

    /**
     * @param intakeSubsystem Intake alt sistemi
     * @param voltage         Uygulanacak voltaj (pozitif = içe çek)
     */
    public RunIntakeCommand(IntakeSubsystem intakeSubsystem, double voltage) {
        this.intakeSubsystem = intakeSubsystem;
        this.voltage = voltage;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runRoller(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runRoller(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
