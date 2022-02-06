// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import java.util.Map;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;


/**
 * This class represents the binding between a specific {@link Action} and a given
 * channel on a particular {@link GenericHID}.
 * @param <TAction> The subtype of action
 */
public abstract class ActionBinding<TAction extends Action> {
  private static final Logger log = LogManager.getLogger(ActionBinding.class);

  protected final TAction m_action;
  protected final OISubsystem m_oi;

  private GenericHID m_boundController;
  private int m_boundChannel;

  public ActionBinding(TAction axis, OISubsystem oi, ShuffleboardContainer container) {
    m_action = axis;
    m_oi = oi;

    setupDefaultBinding();

    var grid = container.getLayout(m_action.getName(), BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withProperties(Map.of(
            "Number of Columns", 2,
            "Number of Rows", 1,
            "Label position", "HIDDEN"));

    var bindButton = grid.add("Bind", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    grid.addString("label", this::toString);

    new NetworkButton(bindButton).whileHeld(new BindAction(this, m_oi, () -> bindButton.setBoolean(false)));

    log.debug("ShuffleBoard configured for action {}", axis::getName);
  }

  /**
   * Get a human readable name for the specified channel (ie an axis or button name).
   * Does NOT include the controller type or port
   * @return Name of the specified channel
   */
  public abstract String getChannelName(int channel);

  /**
   * Get the channel (ie axis/button number) currently being selected by the user.
   * If multiple channels are being selected, one may be chosen arbitrarily.
   * @return The selected channel number from the joystick or null if none selected
   */
  public abstract Integer getSelectedChannel(GenericHID joystick);

  /**
   * @return The axis being bound
   */
  public TAction getAction() {
    return m_action;
  }

  /**
   * @return The name of the axis being bound
   */
  public String getActionName() {
    return m_action.getName();
  }

  /**
   * @return The {@link GenericHID} this action is currently bound to
   */
  public GenericHID getBoundController() {
    return m_boundController;
  }

  /**
   * @return The channel on the {@link this#getBoundController()}
   */
  public int getBoundChannel() {
    return m_boundChannel;
  }

  /**
   * Bind this action to a new channel
   * @param joystick The joystick to bind to
   * @param channelNum The channel number to bind to
   */
  public void bindTo(GenericHID joystick, int channelNum) {
    m_boundController = joystick;
    m_boundChannel = channelNum;

    log.info("Successfully bound {} to {}", m_action::getName, this::toString);
  }


  private void setupDefaultBinding() {
    if (m_boundController == null) {
      var joysticks = m_oi.getJoysticks();
      var defaultJoystick = joysticks.get(0);
      for (GenericHID joystick : joysticks) {
        if (joystick.getPort() == m_action.getDefaultPort()) {
          defaultJoystick = joystick;
          break;
        }
      }

      bindTo(defaultJoystick, m_action.getDefaultChannel());
    }
  }

  /**
   * Convert current binding to an easy to read format
   *
   * @return A human readable string representation of the current binding value
   */
  @Override
  public String toString() {
    return String.format(
            "%s %d: %s",
            m_boundController.getClass().getSimpleName(),
            m_boundController.getPort(),
            getChannelName(m_boundChannel)
    );
  }
}
