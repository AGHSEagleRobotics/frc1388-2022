// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Drive extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem; 
  //private CompdashBoard m_compdashboard; 
  private Supplier<Double> m_driveLeftStickYAxis; 
  private Supplier<Double> m_driveRightStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;

  /** Creates a new Drive. */
  public Drive(
    DriveTrainSubsystem driveTrainSubsystem, 
    //CompdashBoard compdashboard, 
    Supplier<Double> driveLeftStickYAxis, 
    Supplier<Double> driveRightStickYAxis,
    Supplier<Double> driveRightStickXAxis)
     {
    // Use addRequirements() here to declare subsystem dependencies. 

    m_driveTrainSubsystem = driveTrainSubsystem;
    m_driveLeftStickYAxis = driveLeftStickYAxis;
    m_driveRightStickYAxis = driveRightStickYAxis;
    m_driveRightStickXAxis = driveRightStickXAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveTrainSubsystem.curvatureDrive(m_driveLeftStickYAxis.get(), m_driveRightStickXAxis.get(), true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
