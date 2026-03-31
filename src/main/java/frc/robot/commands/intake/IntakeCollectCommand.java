package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Standard intake collection command that deploys the intake,
 * runs the rollers at a given RPM, and runs the feeder to pull in notes.
 * Used in both Teleop and Autonomous.
 */
public class IntakeCollectCommand extends Command {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final double rollerRPM;

    /**
     * Creates a new IntakeCollectCommand with default 2000 RPM.
     */
    public IntakeCollectCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
        this(intake, feeder, 2000.0);
    }

    /**
     * Creates a new IntakeCollectCommand with custom RPM.
     */
    public IntakeCollectCommand(IntakeSubsystem intake, FeederSubsystem feeder, double rollerRPM) {
        this.intake = intake;
        this.feeder = feeder;
        this.rollerRPM = rollerRPM;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        intake.deploy();
    }

    @Override
    public void execute() {
        intake.runRollerRPM(rollerRPM);
        feeder.feedSlow();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
        intake.retract();
        feeder.stop();
    }
}
