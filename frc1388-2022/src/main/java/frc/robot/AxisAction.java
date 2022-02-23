package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import com.eaglerobotics.lib.shuffleboard.InputAction;

public enum AxisAction implements InputAction {
    
    LEFT_DRIVE("Left Drive", null, 0, XboxController.Axis.kLeftY.value),
    RIGHT_DRIVE("Right Drive", null, 0, XboxController.Axis.kRightY.value);

    private final String m_label;
    private final String m_description;
    private final Integer m_defaultAxis;
    private final Integer m_defaultPort;

    AxisAction(String name, String description, Integer defaultPort, Integer defaultAxis) {
        m_label = name;
        m_description = description;
        m_defaultAxis = defaultAxis;
        m_defaultPort = defaultPort;
    }

    @Override
    public String getLabel() {
        return m_label;
    }

    @Override
    public String getDescription() {
        return m_description;
    }

    @Override
    public Integer getDefaultChannel() {
        return m_defaultAxis;
    }

    @Override
    public Integer getDefaultPort() {
        return m_defaultPort;
    }
}
