package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ControllerBindings<T extends AxisEnum> {

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Controls");

    private final Map<Integer, GenericHID> m_controllers;

    private final Map<T, SendableChooser<Integer>> m_bindingChoosers;

    public ControllerBindings(Class<T> axisEnumType, GenericHID... controllers) {
        // Convert controller list into map of port to controller (for easy lookup)
        m_controllers = new HashMap<>();
        for (GenericHID controller : controllers) {
            m_controllers.put(controller.getPort(), controller);
        }

        // build widgets for setting axis bindings
        m_bindingChoosers = new HashMap<>();
        for (T axis : axisEnumType.getEnumConstants()) {

            // Create chooser
            var chooser = new SendableChooser<Integer>();
            m_bindingChoosers.put(axis, chooser);
            
            // Add options for each axis
            for(int i = 0; i < controllers[0].getAxisCount(); i++) {
                chooser.addOption(Integer.toString(i), i);
            }

            // Set default if we have one
            if (axis.getDefaultAxis() != null) {
                chooser.setDefaultOption(axis.getDefaultAxis().toString(), axis.getDefaultAxis());
            }

            // Add choser to tab
            m_tab.add(axis.getName(), chooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
        }
    }

    public double getAxisValue(T axis) {
        var chooser = m_bindingChoosers.get(axis);

        var selectedAxis = chooser.getSelected();

        return m_controllers.get(0).getRawAxis(selectedAxis);
    }
}
