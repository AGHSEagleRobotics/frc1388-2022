// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ClimberCommandConstants;
import frc.robot.Constants.ClimberConstants.ArticulatorPositions;

public class ClimberCommand extends CommandBase {

  private final ClimberSubsystem m_climberSubsystem;
  private final Supplier<Double> m_extendAxis;
  private final Supplier<Double> m_articulateAxis;
  private final Supplier<Boolean> m_articulatePositionVertical;
  private final Supplier<Boolean> m_articulatePositionReach;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(
    ClimberSubsystem climberSubsystem,

    //CompdashBoard compdashboard, 
    Supplier<Double> extendAxis,
    Supplier<Double> articulateAxis,
    Supplier<Boolean> articulatePositionVertical,
    Supplier<Boolean> articulatePositionReach
  ) {

    m_climberSubsystem = climberSubsystem;
    m_extendAxis = extendAxis;
    m_articulateAxis = articulateAxis;
    m_articulatePositionVertical = articulatePositionVertical;
    m_articulatePositionReach  = articulatePositionReach;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double deadband = MathUtil.applyDeadband(m_extendAxis.get(), ClimberCommandConstants.DEADBAND);
    if ( deadband == 0) {
      m_climberSubsystem.setWinchPower(0);
    } else {
      double speed = Math.copySign(Math.pow(deadband, 2) * ClimberCommandConstants.MAX_WINCH_SPEED, deadband);
      m_climberSubsystem.setWinchSpeed(speed);
    }

    deadband = MathUtil.applyDeadband(m_articulateAxis.get(), ClimberCommandConstants.DEADBAND);
    System.out.println("deadband" + deadband);
    SmartDashboard.putNumber("articulator input", deadband);
    if (deadband != 0) {
      m_climberSubsystem.setArticulatorPower(Math.copySign(Math.pow(deadband, 2), deadband));
    } else {
      if (m_articulatePositionVertical.get()) {
        m_climberSubsystem.setArticulatorVertical(); //m_climberSubsystem.setArticulatorPosition(ArticulatorPositions.VERTICAL);
      } else if (m_articulatePositionReach.get()) {
        m_climberSubsystem.setArticulatorReach();    //m_climberSubsystem.setArticulatorPosition(ArticulatorPositions.REACH);
      }
    }

    if (m_climberSubsystem.getArticulatorVerticalLimit()) {
      m_climberSubsystem.setArticulatorPower(0);
    }


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
