package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.XboxController;

public enum AxisEnum implements Action {
    
    LEFT_DRIVE("Left Drive", null, 0, XboxController.Axis.kLeftY.value),
    RIGHT_DRIVE("Right Drive", null, 0, XboxController.Axis.kRightY.value);

    private final String m_name;
    private final String m_description;
    private final Integer m_defaultAxis;
    private final Integer m_defaultPort;

    AxisEnum(String name, String description, Integer defaultPort, Integer defaultAxis) {
        m_name = name;
        m_description = description;
        m_defaultAxis = defaultAxis;
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
        return m_defaultAxis;
    }

    @Override
    public Integer getDefaultPort() {
        return m_defaultPort;
    }
}
