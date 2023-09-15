// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

public class REject extends CommandBase {
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterFeederSubsystem m_shooterFeederSubsystem;
  private final TransitionSubsystem m_transitionSubsystem; 
  /** Creates a new ReverseShootEject. */
  public REject(IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem, ShooterFeederSubsystem shooterFeederSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterFeederSubsystem = shooterFeederSubsystem;
    m_transitionSubsystem = transitionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SPEED_DEPLOY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeWheelSpin(-IntakeConstants.WHEEL_SPEED_INTAKE);
    //or m_intakeSubsystem.setIntakeArmMotor(-IntakeConstants.ARM_SPEED_DEPLOY);
    m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.REVERSE);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_REVERSE_FAST);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turn off feeder motor
    m_shooterFeederSubsystem.setFeederFunction(FeederFunctions.OFF);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
    m_intakeSubsystem.setIntakeWheelSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
