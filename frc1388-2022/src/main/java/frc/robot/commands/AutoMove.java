// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMove extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_setPoint;
  private final double m_speed;
  private final double m_rotation;
  private final Timer m_timer = new Timer();

  private final PIDController m_pidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  //do I need a mode for this?
  public enum mode {
    kTimeDrive, kDistanceDrive
  }

  /** Creates a new AutoMove. */
  public AutoMove(DriveTrainSubsystem driveTrainSubsystem, double setPoint, double speed, double rotation) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    m_rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidController.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_driveTrainSubsystem.resetLeftEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrainSubsystem.curvatureDrive(m_speed, m_rotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_pidController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
