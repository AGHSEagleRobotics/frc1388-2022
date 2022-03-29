// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class DeployIntake extends CommandBase {

  private final IntakeSubsystem m_intakeSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;

  /** Creates a new IntakeCommand. */
  public DeployIntake (IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem) { 
    m_intakeSubsystem = intakeSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeArmMotor(IntakeConstants.ARM_SPEED_DEPLOY);
    m_intakeSubsystem.setIntakeWheelSpin(IntakeConstants.WHEEL_SPEED_INTAKE);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_MEDIUM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SPEED_DEPLOY);
    m_intakeSubsystem.setIntakeArmMotor(0);
   // m_intakeSubsystem.setIntakeWheelSpin(0);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isLimitDownReached();
  }
}
