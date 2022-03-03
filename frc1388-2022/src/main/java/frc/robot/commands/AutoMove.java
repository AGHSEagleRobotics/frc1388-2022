// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMove extends CommandBase {

  private static final Logger log = LogManager.getLogger(AutoMove.class);
  
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
  public AutoMove(DriveTrainSubsystem driveTrainSubsystem, double setPoint, double speed) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_setPoint = setPoint;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);

    //Setting PID control tolerance
    m_pidController.setTolerance(AutoMoveConstants.P_TOLERANCE); //change P tolerance?
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

    speed = m_pidController.calculate(leftEncoderDistance, m_setPoint);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);

    m_driveTrainSubsystem.curvatureDrive(speed, 0, false);

    log.info("Distance: {} \tspeed: {} \tsetPoint: {}", leftEncoderDistance, speed, m_setPoint); 
  
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
