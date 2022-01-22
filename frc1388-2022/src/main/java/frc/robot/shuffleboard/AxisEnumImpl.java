package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.XboxController;

public enum AxisEnumImpl implements AxisEnum {
    
    LEFT_DRIVE("Left Drive", null, XboxController.Axis.kLeftY.value),
    RIGHT_DRIVE("Right Drive", null, XboxController.Axis.kRightY.value);

    private final String name;
    private final String description;
    private final Integer defaultAxis;

    private AxisEnumImpl(String name, String description, Integer defaultAxis) {
        this.name = name;
        this.description = description;
        this.defaultAxis = defaultAxis;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getDescription() {
        return description;
    }

    @Override
    public Integer getDefaultAxis() {
        return defaultAxis;
    }    
}
