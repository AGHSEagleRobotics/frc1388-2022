// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterFeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterTargetRPM extends InstantCommand {

  //THIS is a development tool
  private final ShooterFeederSubsystem m_shooter;
  private final double m_targetRPM;

  public SetShooterTargetRPM(ShooterFeederSubsystem shooter, double targetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setTargetRPM(m_targetRPM);
  }

}
