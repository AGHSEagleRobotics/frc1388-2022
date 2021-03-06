// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ClimberCommandConstants;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberRetract extends CommandBase {

//   private final ClimberSubsystem m_climberSubsystem;
//   private int timer = 0;

//   /** Creates a new ClimberRetract. */
//   public ClimberRetract(ClimberSubsystem climberSubsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_climberSubsystem = climberSubsystem;
//     addRequirements(m_climberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climberSubsystem.resetWinchLimit();
//     timer += 1;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_climberSubsystem.setArticulatorPower(0);
//     m_climberSubsystem.setWinchPower(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (timer > 100);
//   }
// }
