package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import com.eaglerobotics.lib.shuffleboard.Action;

public enum ButtonAction implements Action {
  DEPLOY_INTAKE("Deploy Intake", null, 0, XboxController.Button.kA.value),
  RETRACT_INTAKE("Retract Intake", null, 0, XboxController.Button.kB.value);

  private final String m_name;
  private final String m_description;
  private final Integer m_defaultButton;
  private final Integer m_defaultPort;

  ButtonAction(String name, String description, Integer defaultPort, Integer defaultButton) {
    m_name = name;
    m_description = description;
    m_defaultButton = defaultButton;
    m_defaultPort = defaultPort;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public String getDescription() {
    return m_description;
  }

  @Override
  public Integer getDefaultChannel() {
    return m_defaultButton;
  }

  @Override
  public Integer getDefaultPort() {
    return m_defaultPort;
  }
}
