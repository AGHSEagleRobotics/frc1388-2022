// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;


/**
 * Class that manages the ShuffleBoard aspects of binding a specifc Axis provided
 * in {@link AxisEnum} to an particular joystick port and axis
 * 
 * Also includes some helper functions for retriving the current value of the bound
 * axis from the joystick
 */
public class AxisBinding<T extends AxisEnum> implements Supplier<Double> {

  private static final Logger log = LogManager.getLogger(AxisBinding.class);

  private final T m_axis;
  private final OISubsystem m_oi;

  private GenericHID m_boundController;
  private int m_boundAxis;

  public AxisBinding(T axis, OISubsystem oi, ShuffleboardContainer container) {
    m_axis = axis;
    m_oi = oi;

    if (oi.getJoysticks().size() == 0) {
      throw new IllegalArgumentException("Must provide at least one GenericHID");
    }

    m_boundController = oi.getJoysticks().get(0);
    for (GenericHID controller : oi.getJoysticks()) {
      if (controller.getPort() == m_axis.getDefaultPort()) {
        m_boundController = controller;
        break;
      }
    }

    m_boundAxis = m_axis.getDefaultAxis();

    var grid = container.getLayout(m_axis.getName(), BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withProperties(Map.of(
            "Number of Columns", 2,
            "Number of Rows", 1,
            "Label position", "HIDDEN"));

    var bindButton = grid.add("Bind", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    grid.addString("label", this::toString);

    var bindCommand = new BindAxis<>(this, oi, () -> bindButton.setBoolean(false));

    new NetworkButton(bindButton).whileHeld(bindCommand);

    log.debug("Binding configured for {}", axis::getName);
  }

  /**
   * @return The axis being bound
   */
  public T getAxis() {
    return m_axis;
  }

  /**
   * @return The name of the axis being bound
   */
  public String getAxisName() {
    return m_axis.getName();
  }

  public void setBoundAxis(GenericHID joystick, int axisNum) {
    if (!m_oi.getJoysticks().contains(joystick)) {
      throw new IllegalArgumentException("joystick must be part of the OISubsystem");
    }

    log.info("Bound {} to {}:{}", m_axis::getName, joystick::getPort, () -> axisNum);

    m_boundController = joystick;
    m_boundAxis = axisNum;
  }

  /**
   * Reads the value of the bound controller axis
   * 
   * @return The bound controller axis value
   */
  @Override
  public Double get() {
    return m_boundController.getRawAxis(m_boundAxis);
  }

  private static final Map<Class<? extends GenericHID>, Function<Integer, String>> axisPrettifiers = Map.of(

      XboxController.class, axisNum -> Stream.of(XboxController.Axis.values())
          .filter(axis -> axis.value == axisNum)
          .findAny()
          .map(axis -> axis.toString())
          .orElse(axisNum.toString()),

      PS4Controller.class, axisNum -> Stream.of(PS4Controller.Axis.values())
          .filter(axis -> axis.value == axisNum)
          .findAny()
          .map(axis -> axis.toString())
          .toString());

  /**
   * Convert current binding to an easy to read format
   * 
   * @return A human readable string representation of the current binding value
   */
  @Override
  public String toString() {

    var prettifier = axisPrettifiers.getOrDefault(
        m_boundController.getClass(),
        axisNum -> axisNum.toString());

    var axisName = prettifier.apply(m_boundAxis);

    return String.format(
        "%s %d: %s",
        m_boundController.getClass().getSimpleName(),
        m_boundController.getPort(),
        axisName);
  }
}
