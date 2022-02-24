// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.System.LoggerFinder;
import java.lang.System.Logger.Level;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import org.apache.logging.log4j.core.config.Loggers;
import org.apache.logging.log4j.core.jackson.Log4jYamlObjectMapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.Constants.USBConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMove extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Mode m_mode;
  private final double m_setPoint;
  private final double m_speed;
  private final Timer m_timer = new Timer();
  //tried added static to timer, didn't work

  private final PIDController m_pidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  //do I need a mode for this?
  public enum Mode {
    kTimeDrive, kDistanceDrive
  }

  /*int Timer() {
    return 2;
  } */

  /** Creates a new AutoMove. */
  public AutoMove(DriveTrainSubsystem driveTrainSubsystem, double setPoint, double speed, Mode mode) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidController.setTolerance(0.5); //FIXME magic number
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
    double speed;
    double leftEncoderDistance = m_driveTrainSubsystem.getLeftEncoderDistance();
    speed = m_pidController.calculate(leftEncoderDistance);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);

    m_driveTrainSubsystem.curvatureDrive(speed, speed, false);

    // Robot.log.info(); FIXME
  
    /* getRequirements();
    if (m_timer > AUTON_PERIOD?) */
    //trying to stop robot after period competed
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
    return m_timer.hasElapsed(m_setPoint);
  
    //return m_pidController.atSetpoint();
  }
}
