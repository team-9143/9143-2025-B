package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KitBotConstants;
import frc.robot.subsystems.KitBot;

/**
 * A command that ejects a stacked piece by running the roller at ROLLER_STACKED_EJECT_VALUE
 * for 0.5 seconds.
 */
public class EjectStackedPieceCommand extends Command {
    private final KitBot m_kitbot;
    private final double m_duration = 0.5; // seconds
    private long m_startTime;

    /**
     * Creates a new EjectStackedPieceCommand.
     *
     * @param kitbot The KitBot subsystem
     */
    public EjectStackedPieceCommand(KitBot kitbot) {
        m_kitbot = kitbot;
        addRequirements(kitbot);
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
        KitBot.rollerMotor.set(KitBotConstants.ROLLER_STACKED_EJECT_VALUE);
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
        // End the command after 0.5 seconds
        return (System.currentTimeMillis() - m_startTime) >= (m_duration * 1000);
    }
}