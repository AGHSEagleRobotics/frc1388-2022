// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OISubsystem extends SubsystemBase {

  private static final double AXIS_DEADBAND = 0.5;

  private final List<GenericHID> m_joysticks;

  public OISubsystem(GenericHID... joysticks) {
    if (joysticks.length == 0) {
      throw new IllegalArgumentException("Must provide at least one joystick ");
    }
    m_joysticks = Arrays.asList(joysticks);
  }

  public List<GenericHID> getJoysticks() {
    return m_joysticks;
  }

}
