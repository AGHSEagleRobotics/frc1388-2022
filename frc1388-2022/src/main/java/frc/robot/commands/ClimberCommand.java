// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;import frc.robot.Constants.ClimberCommandConstants;
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
    /*
    double deadband = MathUtil.applyDeadband(m_extendAxis.get(), ClimberCommandConstants.DEADBAND);
    if ( deadband == 0) {
      m_climberSubsystem.setWinchPower(0);
    } else {
      double speed = Math.copySign(Math.pow(deadband, 2), deadband);
      m_climberSubsystem.setWinchPower(speed);
    }

    deadband = MathUtil.applyDeadband(m_articulateAxis.get(), ClimberCommandConstants.DEADBAND);
    SmartDashboard.putNumber("articulator input", deadband);
    m_climberSubsystem.setArticulatorPower(Math.copySign(Math.pow(deadband, 2), deadband));
    */
    // extend is negated!!
    
    m_climberSubsystem.setWinchPower(MathUtil.applyDeadband(m_extendAxis.get(), ClimberCommandConstants.DEADBAND));
    m_climberSubsystem.setArticulatorPower(Math.copySign(Math.pow(m_articulateAxis.get(), 2), m_articulateAxis.get()));
    // m_climberSubsystem.setArticulatorPower(MathUtil.applyDeadband(m_articulateAxis.get(), ClimberCommandConstants.DEADBAND));
    
    // FIXME
    // if (m_climberSubsystem.isArticulatorAtVerticalLimit()) {
    //   m_climberSubsystem.setArticulatorPower(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops motors when command is interupted
    m_climberSubsystem.setWinchPower(0);
    m_climberSubsystem.setArticulatorPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
