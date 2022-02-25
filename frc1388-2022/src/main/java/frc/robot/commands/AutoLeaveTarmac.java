// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoLeaveTarmac extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_setPoint;
  private final double m_speed;
  private final Timer m_timer = new Timer();
  //tried added static to timer, didn't work

  private final PIDController m_pidController = new PIDController(AutoMoveConstants.P_VALUE, 0, 0);

  /*int Timer() {
    return 2;
  } */

  /** Creates a new AutoMove. */
  public AutoLeaveTarmac(DriveTrainSubsystem driveTrainSubsystem, double setPoint, double speed) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidController.setTolerance(AutoMoveConstants.P_TOLERANCE); //FIXME magic number
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
    double speed = m_speed;
    double setPoint = m_setPoint;
    double leftEncoderDistance = m_driveTrainSubsystem.getLeftEncoderDistance();

    speed = m_pidController.calculate(leftEncoderDistance, setPoint);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);

    m_driveTrainSubsystem.curvatureDrive(speed, speed, false);

    //Robot.log.info(); TODO need to change log visibility to public
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_pidController.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_pidController.atSetpoint() || (Timer.getMatchTime() > AutoMoveConstants.AUTO_TIME) );
    return finished;
  }
}
