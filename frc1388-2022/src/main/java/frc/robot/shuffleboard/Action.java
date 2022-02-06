// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

public interface Action {
    String getName();
    String getDescription();

    Integer getDefaultChannel();
    Integer getDefaultPort();
}
