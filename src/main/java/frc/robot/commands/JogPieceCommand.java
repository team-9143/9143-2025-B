package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KitBotConstants;
import frc.robot.subsystems.KitBot;

/**
 * A command that jogs a piece by running the roller slowly.
 * The command runs until canceled.
 */
public class JogPieceCommand extends Command {
    private final KitBot m_kitbot;

    /**
     * Creates a new JogPieceCommand.
     *
     * @param kitbot The KitBot subsystem
     */
    public JogPieceCommand(KitBot kitbot) {
        m_kitbot = kitbot;
        addRequirements(kitbot);
    }

    @Override
    public void initialize() {
        KitBot.rollerMotor.set(KitBotConstants.ROLLER_JOG_VALUE);
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