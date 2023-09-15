// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.REVPhysicsSim;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.GuestMode;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final Logger log = LogManager.getLogger(Robot.class);

  private Command m_autonomousCommand;
  private Command m_retractClimberWinchCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    log.info("robotInit");

    // log and print software version
    log.info("Git version: " + BuildInfo.GIT_VERSION + " (branch: " + BuildInfo.GIT_BRANCH + "" + BuildInfo.GIT_STATUS + ")");
    log.info("      Built: " + BuildInfo.BUILD_DATE + "  " + BuildInfo.BUILD_TIME);
    //Remember this silences joystick warnings
    DriverStation.silenceJoystickConnectionWarning(true);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData(CommandScheduler.getInstance());
    m_robotContainer.setNeutralMode(NeutralMode.Coast);

    CommandScheduler.getInstance().onCommandInitialize(command -> log.info("++ " + command.getName() + " Initialized" ));
    CommandScheduler.getInstance().onCommandInterrupt(command -> log.info("-- " + command.getName() + " Interrupted" ));
    CommandScheduler.getInstance().onCommandFinish(command -> log.info("-- " + command.getName() + " Finished" ));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (RobotController.getUserButton()) {
      m_robotContainer.setNeutralMode(NeutralMode.Coast);
      System.out.println("###RobotPeriodic() -> UserButtonPressed -> NeutralMode.Coast###");
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setNeutralMode(NeutralMode.Brake);
    log.info("########  Robot disabled");
    m_robotContainer.m_guestMode.setGuestMode(false);
  }
  

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link m_robotContainer} class. */
  @Override
  public void autonomousInit() {
    log.info("########  Autonomous enabled");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.setNeutralMode(NeutralMode.Brake);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      m_robotContainer.m_guestMode.setGuestMode(false);
    }

    
    // Get match info from FMS
    if (DriverStation.isFMSAttached()) {
      String fmsInfo = "FMS info: ";
      fmsInfo += " " + DriverStation.getEventName();
      fmsInfo += " " + DriverStation.getMatchType();
      fmsInfo += " match " + DriverStation.getMatchNumber();
      fmsInfo += " replay " + DriverStation.getReplayNumber();
      fmsInfo += ";  " + DriverStation.getAlliance() + " alliance";
      fmsInfo += ",  Driver Station " + DriverStation.getLocation();
      log.info(fmsInfo);
    } else {
      log.info("FMS not connected");
    //m_retractClimberWinchCommand = m_robotContainer.getRetractCommand();
    // m_retractClimberWinchCommand.schedule();

    log.info("Match type:\t" + DriverStation.getMatchType());
    log.info("Event name:\t" + DriverStation.getEventName());
    log.info("Alliance:\t" + DriverStation.getAlliance());
    log.info("Match number:\t" + DriverStation.getMatchNumber());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    log.info("########  Teleop enabled");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_guestMode.setGuestMode(false);
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
  
  @Override
  public void testInit() {
    log.info("########  Test enabled");

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_guestMode.setGuestMode(false);
  }
  
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {
    log.info("########  Simulation enabled");
    
    m_robotContainer.simulationInit(); 
    // m_robotContainer.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_guestMode.setGuestMode(false);
  }

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
