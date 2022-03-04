// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoMoveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommands extends SequentialCommandGroup {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final ShooterFeederSubsystem m_shooterFeederSubsystem;
  /** Creates a new AutonomousCommands. */
  public AutonomousCommands(DriveTrainSubsystem driveTrainSubsystem, ShooterFeederSubsystem shooterFeederSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_shooterFeederSubsystem = shooterFeederSubsystem;

    Command autoMove = new AutoMove(m_driveTrainSubsystem, 
    AutoMoveConstants.AUTO_TARMAC_DISTANCE,    
    AutoMoveConstants.AUTO_DRIVE_SPEED);
    //Set these commands up
    Command autoTurn = new AutoTurn(m_driveTrainSubsystem, 0); //TODO CHANGE
    Command autoShoot = new AutoShoot(m_shooterFeederSubsystem, ShooterConstants.SHOOTER_RMP_LOWGOAL);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(autoShoot, autoMove);
  }
}
