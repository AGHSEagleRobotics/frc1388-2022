package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;


/**
 * Command that will listen for the first channel to be activated (ie button pressed/axis moved)
 * on any connected controller.
 */
public class BindAction extends CommandBase {

  private static final Logger log = LogManager.getLogger(BindAction.class);

  private final OISubsystem m_oiSubsystem;

  private final ActionBinding<?> m_binding;
  private final Runnable m_finalize;

  private boolean m_done = false;

  public BindAction(ActionBinding<?> binding, OISubsystem oiSubsystem, Runnable finalize) {
    m_oiSubsystem = oiSubsystem;
    m_binding = binding;
    m_finalize = finalize;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(oiSubsystem);
  }

  @Override
  public void initialize() {
    log.debug("Rebinding {}. Waiting for input", m_binding::getActionName);
    m_done = false;
  }

  @Override
  public void execute() {
    for (GenericHID joystick: m_oiSubsystem.getJoysticks()) {
      var selectedChannel = m_binding.getSelectedChannel(joystick);
      if (selectedChannel != null) {
        m_binding.bindTo(joystick, selectedChannel);

        m_done = true;
        return;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_done;
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allow rebinding while robot is disabled.
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_finalize.run();
  }
}
