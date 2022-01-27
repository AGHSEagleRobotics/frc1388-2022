// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {

  private final ClimberSubsystem m_climberSubsystem;
  private final Supplier<Double> m_extendAxis;
  private final Supplier<Double> m_articulateAxis;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(
    ClimberSubsystem climberSubsystem,
    //CompdashBoard compdashboard, 
    Supplier<Double> extendAxis,
    Supplier<Double> articulateAxis
  ) {

    m_climberSubsystem = climberSubsystem;
    m_extendAxis = extendAxis;
    m_articulateAxis = articulateAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_climberSubsystem.setWinchPower(m_extendAxis.get());
    m_climberSubsystem.setArticulatorPower(m_articulateAxis.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
