// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that will watch for whenever the first axis to be registered by
 * {@link OISubsystem#findLargestAxis()} and bind that axis the the request action
 */
public class BindAxis<T extends AxisEnum> extends CommandBase {
  private static final Logger log = LogManager.getLogger(BindAxis.class);

  private final AxisBinding<T> m_binding;
  private final OISubsystem m_oi;

  private boolean m_foundBinding = false;

  private Runnable m_end;

  /** Creates a new BindAxis. */
  public BindAxis(AxisBinding<T> binding, OISubsystem oi, Runnable end) {
    m_binding = binding;
    m_oi = oi;
    m_end = end;

    addRequirements(oi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.debug("Bind pressed for {}", m_binding::getAxisName);
    m_foundBinding = false;
  }

  // Called repeatedly while the command is scheduled.
  @Override
  public void execute() {
    var largestAxis = m_oi.findLargestAxis();
    if (largestAxis != null) {
      m_binding.setBoundAxis(largestAxis.getKey(), largestAxis.getValue());
      m_foundBinding = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_end.run();
    log.debug("BindAxis done for {}. {}", m_binding::getAxisName, () -> interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_foundBinding;
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allow this command to run while disabled
    return true;
  }
}
