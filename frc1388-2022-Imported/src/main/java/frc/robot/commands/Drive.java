// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.RowId;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RumbleConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.RumbleSubsystem;

public class Drive extends CommandBase {

  private DriveTrainSubsystem m_driveTrainSubsystem;
  // private Dashboard m_Dashboard;
  private boolean m_precisionMode = false;
  private boolean m_lastRightStickButton = false;
  private RumbleSubsystem m_precisionRumble;
  private Supplier<Double> m_driveLeftStickYAxis;
  //private Supplier<Double> m_driveRightStickYAxis;
  private Supplier<Double> m_driveRightStickXAxis;

  private Supplier<Boolean> m_driveRightStickButton;

  // limelight
  private Supplier<Double> m_limelightX;
  private Supplier<Double> m_limelightY;
  private Supplier<Double> m_limelightAREA;
  private Supplier<Double> m_limelightV;

  private PIDController m_PidController = new PIDController(1, 0, 0);

  private Supplier<Double> m_trigger;

  /** Creates a new Drive. */
  public Drive(
      DriveTrainSubsystem driveTrainSubsystem,
      RumbleSubsystem precisionrumble,
      // Dashboard dashboard,
      Supplier<Double> driveLeftStickYAxis,
      Supplier<Double> driveRightStickYAxis,
      Supplier<Double> driveRightStickXAxis,
      Supplier<Boolean> driveRightStickButton,
      Supplier<Double> trigger,
      Supplier<Double> LimelightX,
      Supplier<Double> LimelightY,
      Supplier<Double> LimelightAREA,
      Supplier<Double> LimelightV


      ) {
      
        m_limelightX = LimelightX;
        m_limelightY = LimelightY;
        m_limelightAREA = LimelightAREA;
        m_limelightV = LimelightV;

        m_trigger = trigger;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);

    m_driveTrainSubsystem = driveTrainSubsystem;
    m_precisionRumble = precisionrumble;
    m_driveLeftStickYAxis = driveLeftStickYAxis;
    //m_driveRightStickYAxis = driveRightStickYAxis;
    m_driveRightStickXAxis = driveRightStickXAxis;
    m_driveRightStickButton = driveRightStickButton;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrainSubsystem.setDeadbandDefault();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // For tank drive
    //double leftSpeed = -m_driveLeftStickYAxis.get();
    //double rightSpeed = -m_driveRightStickYAxis.get();

    // checks to see if the button has been pressed and then flags the precision mode
    // Don't trigger again if the button is continually held


    // boolean rightStickButton = m_driveRightStickButton.get();
    // if (rightStickButton && !m_lastRightStickButton ) {
    //   m_precisionMode = !m_precisionMode;
    //   if (m_precisionMode) {

    //     //TODO change left to right - ask driver
    //     m_precisionRumble.rumblePulse(RumbleConstants.RumbleSide.RIGHT);
    //   } else {
    //     m_precisionRumble.rumblePulse(RumbleConstants.RumbleSide.LEFT);
    //   }
    // }


    // Save the value to be compared on the next tick
    // m_lastRightStickButton = rightStickButton;

    // When in precision mode, scale the turning precision


    // if (m_precisionMode) {
    //   rotation = scale(rotation);
    // }

    // scale the drive speed
    // speed = scale(speed);
    
    // One of three drives to choose from


    // m_driveTrainSubsystem.curvatureDrive(speed, rotation, m_precisionMode);


    // m_driveTrainSubsystem.tankDrive(leftSpeed, rightSpeed);

    double speed = 0;
    double rotation = 0;
    double llx = m_limelightX.get();

    if ((m_trigger.get() > .5) && (m_limelightV.get() == 1.0)) { // if we are going to a target

      // rotation = MathUtil.clamp(0.05 * llx, -0.4, 0.4);
      // rotation = MathUtil.clamp(llx, -0.4, 0.4);
      // if (Math.abs(m_limelightX.get()) < 5.0) {
      // }

      rotation = llx;
      rotation /= 30.0;
      if (Math.abs(rotation) < 0.34) rotation = Math.copySign(0.34, rotation);
      rotation = MathUtil.clamp(rotation, -0.45, 0.45);
      if (Math.abs(llx) < 1.5) rotation = 0.0;

      speed = -m_driveLeftStickYAxis.get(); 

    } else {
      speed = -m_driveLeftStickYAxis.get(); 
      rotation = m_driveRightStickXAxis.get();
    }
    if (Math.random() > 0.9)System.out.println("rotation: " + rotation + "\t\ttarget x: " + Math.round(llx));
    m_driveTrainSubsystem.arcadeDrive(speed, -rotation);

  }

  public double scale(double input) {
    // squaring input
    return Math.copySign(input * input, input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
