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
    Supplier<Double> driveRightStickXAxis
  ) {
    addRequirements(driveTrainSubsystem);
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

    double speed = -m_driveLeftStickYAxis.get();
    double rotation = m_driveRightStickXAxis.get();
    
    double leftSpeed = -m_driveLeftStickYAxis.get();
    double rightSpeed = -m_driveRightStickYAxis.get();
    
    //One of three drives to choose from
    m_driveTrainSubsystem.curvatureDrive( speed, rotation, true);
    // m_driveTrainSubsystem.arcadeDrive(speed, rotation);
    // m_driveTrainSubsystem.tankDrive(leftSpeed, rightSpeed);

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
