// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends CommandBase {

  private final IntakeSubsystem m_intakeSubsystem;
  /**after the limit switch is initaly triggered, the motor will run at low power for a bit longer to ensure limit switch is triggered.*/
  private boolean m_intakeLimitHitOnce = false;
  private int m_lowPowerTimer = 0;
  private boolean m_isfinished = false;

  

  /** Creates a new IntakeCommand. */
  public RetractIntake (IntakeSubsystem intakeSubsystem) { 
    m_intakeSubsystem = intakeSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem); 
  }

  /*
   * arm motor needs to deploy 
   * wheel motor needs to run forward
   * arm motor needs a limit 
   */

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeLimitHitOnce = false;
    m_lowPowerTimer = 0;
    m_isfinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.isLimitUpReached()) {
      m_intakeLimitHitOnce = true;
    }
    
    if (!m_intakeLimitHitOnce) {
      m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SPEED_DEPLOY);
    } else {
      if (++m_lowPowerTimer <= IntakeConstants.ARM_SLOW_SPEED_TICKS) {
        m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SLOW_DEPLOY);
      } else {
        m_isfinished = true;
      }
    }

    if (m_intakeSubsystem.isCloseToUpLimit()) {
      m_intakeSubsystem.setIntakeWheelSpin(0);
      m_intakeSubsystem.setIntakeArmMotor(-0.5); // IntakeConstants.ARM_SLOW_DEPLOY);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SPEED_DEPLOY);
      m_intakeSubsystem.setIntakeWheelSpin(0);
      m_intakeSubsystem.setIntakeArmMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isfinished;
  }
}
