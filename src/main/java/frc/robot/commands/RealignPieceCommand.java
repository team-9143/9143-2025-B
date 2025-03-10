package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KitBotConstants;
import frc.robot.subsystems.KitBot;

/**
 * A command that realigns a piece by running the roller in reverse.
 * The command runs until canceled.
 */
public class RealignPieceCommand extends Command {
    private final KitBot m_kitbot;

    /**
     * Creates a new RealignPieceCommand.
     *
     * @param kitbot The KitBot subsystem
     */
    public RealignPieceCommand(KitBot kitbot) {
        m_kitbot = kitbot;
        addRequirements(kitbot);
    }

    @Override
    public void initialize() {
        KitBot.rollerMotor.set(-KitBotConstants.ROLLER_FIRST_EJECT_VALUE);
    }

    @Override
    public void execute() {
        // Just keep the motor running
    }

    @Override
    public void end(boolean interrupted) {
        KitBot.rollerMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted
        return false;
    }
}