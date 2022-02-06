package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Top-level class to interact with the ShuffleBoard controlelr bindings
 * 
 * Simply call {@link ControllerBindings#getAxisValue(AxisEnum)} to get the
 * current value of the bound axis
 */
public class ControllerBindings<T extends AxisEnum> {

    private final Map<T, AxisBinding<T>> m_axisBindings;

    public ControllerBindings(Class<T> axisEnumType, OISubsystem oi) {
        m_axisBindings = new HashMap<>();

        ShuffleboardTab tab = Shuffleboard.getTab("Controls");

        var axisTypes = axisEnumType.getEnumConstants();
        var axisBindingList = tab.getLayout("Axis Bindings", BuiltInLayouts.kList)
            .withSize(3, axisTypes.length)
            .withProperties(Map.of("Label Position", "TOP"))
            .withPosition(0, 0);
        for (T axis : axisTypes) {
            m_axisBindings.put(axis, new AxisBinding<>(axis, oi, axisBindingList));
        }
    }

    public double getAxisValue(T axis) {
        return m_axisBindings.get(axis).get();
    }
}
