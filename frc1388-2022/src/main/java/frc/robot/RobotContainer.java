// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.eaglerobotics.lib.shuffleboard.InputActionBindings;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.Constants.DriveTrainConstants; // climber constats
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.USBConstants; // USB
import frc.robot.Constants.ClimberConstants.ArticulatorPositions;
import frc.robot.Constants.DashboardConstants.Cameras;
import frc.robot.InputActions.AxisInputs;
import frc.robot.InputActions.ButtonInputs;
import frc.robot.commands.Drive;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ClimberCommand; // climber command
import frc.robot.subsystems.ClimberSubsystem; // climber subsystem
import frc.robot.subsystems.DriveTrainSubsystem; // drive train subsystem
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // components
  public static XboxController m_driveController = new XboxController(USBConstants.DRIVE_CONTROLLER);
  public static XboxController m_opController = new XboxController(USBConstants.OP_CONTROLLER);

  private final InputActionBindings<AxisInputs, ButtonInputs> m_inputBindings = new InputActionBindings<>(
      AxisInputs.class,
      ButtonInputs.class,
      m_driveController,
      m_opController
  );

  // The robot's subsystems and commands are defined here...

  private final RumbleSubsystem m_rumbleSubsystem = new RumbleSubsystem(m_driveController);

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
      new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT),
      new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK),
      new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT),
      new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK));

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(
      new WPI_TalonFX(ClimberConstants.CANID_WINCH),
      new CANSparkMax(ClimberConstants.CANID_ARTICULATOR, MotorType.kBrushless));

  private final ShooterFeederSubsystem m_shooterSubsystem = new ShooterFeederSubsystem(
      new WPI_TalonFX(ShooterConstants.CANID_SHOOTER_MOTOR),
      new CANSparkMax(ShooterConstants.CANID_FEEDER_MOTOR, MotorType.kBrushless));

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      new CANSparkMax(IntakeConstants.CANID_ARM_MOTOR, MotorType.kBrushless),
      new CANSparkMax(IntakeConstants.CANID_WHEEL_MOTOR, MotorType.kBrushless));

  private final TransitionSubsystem m_transitionSubsystem = new TransitionSubsystem(
      new CANSparkMax(TransitionConstants.CANID_TRANSITION_MOTOR, MotorType.kBrushless));

  private final Dashboard m_dashboard = new Dashboard();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // set default commands
    //REVIEW CLIMBER DEFAULTS IN ROBOT CONTAINER
    m_climberSubsystem.setDefaultCommand(
        new ClimberCommand(
            m_climberSubsystem,
            m_inputBindings.getAxisBinding(AxisInputs.CLIMBER_EXTEND), // extend
            m_inputBindings.getAxisBinding(AxisInputs.CLIMBER_ARTICULATE), // articulate
            () -> m_opController.getYButton(), // vertical (articulate)
            () -> m_opController.getXButton() // reach (articulate)
        ));

    m_driveTrainSubsystem.setDefaultCommand(
        new Drive(
            m_driveTrainSubsystem, m_rumbleSubsystem,
            m_inputBindings.getAxisBinding(AxisInputs.DRIVE_SPEED),
            () -> m_driveController.getRightY(),
            m_inputBindings.getAxisBinding(AxisInputs.DRIVE_TURN),
            //rumble for precision mode
            m_inputBindings.getButtonBinding(ButtonInputs.DRIVE_PRECISION_MODE)));

    m_transitionSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW),
            m_transitionSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {



    /*
     * dev mode
     * new JoystickButton(m_driveController, XboxController.Button.kX.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterRpmStepIncrease());
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kY.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterRpmStepDecrease());
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kA.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterEnabled(true));
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kB.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterEnabled(false));
     * 
     * new JoystickButton(m_opController, XboxController.Button.kRightBumper.value)
      .whileHeld(() -> m_transitionSubsystem.setTransitionSpeed(
            TransitionConstants.TRANSITION_SPEED_REVERSE_SLOW),
            m_transitionSubsystem);
     */


    // INTAKE DEPLOY LEFT TRIGGER
    new Button(() -> isLeftDriverTriggerPressed() || isLeftOpTriggerPressed())
        .whenHeld(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));

    //INTAKE DRIVE UP
    m_inputBindings.getButton(ButtonInputs.RETRACT_INTAKE_DRIVER)
        .whenHeld(new RetractIntake(m_intakeSubsystem));

    // INTAKE OP UP
    m_inputBindings.getButton(ButtonInputs.RETRACT_INTAKE_OP)
        .whenHeld(new RetractIntake(m_intakeSubsystem));

    // SHOOT LOW AND HIGH GOAL
    m_inputBindings.getButton(ButtonInputs.SHOOT_HIGH)
        .whenHeld(new ShootHigh(m_shooterSubsystem, m_transitionSubsystem));

    new Button(RobotContainer::isRightDriverTriggerPressed)
        .whenHeld(new ShootLow(m_shooterSubsystem, m_transitionSubsystem));

    // Lower priority
    m_inputBindings.getButton(ButtonInputs.CLIMBER_REACH)
        .whenPressed(() -> m_climberSubsystem.setArticulatorPosition(ArticulatorPositions.REACH), m_climberSubsystem);

    m_inputBindings.getButton(ButtonInputs.CLIMBER_STOW)
        .whenPressed(() -> m_climberSubsystem.setArticulatorPosition(ArticulatorPositions.VERTICAL),
            m_climberSubsystem);

    // Reverse
    m_inputBindings.getButton(ButtonInputs.DRIVE_FORWARD)
        .whenPressed(() -> setForward(true));

    m_inputBindings.getButton(ButtonInputs.DRIVE_REVERSE)
        .whenPressed(() -> setForward(false));

    //Clean up?
    m_inputBindings.getButton(ButtonInputs.CYCLE_CAMERA_DRIVER)
        .whenPressed(
            new InstantCommand(() -> m_dashboard.switchCamera()) {
              @Override
              public boolean runsWhenDisabled() {
                return true;
              }
            });

    m_inputBindings.getButton(ButtonInputs.CYCLE_CAMERA_OP)
        .whenPressed(
            new InstantCommand(() -> m_dashboard.switchCamera()) {
              @Override
              public boolean runsWhenDisabled() {
                return true;
              }
            });
    // .whenPressed(() -> m_dashboard.switchCamera());
  }

  private void setForward(boolean isForward) {
    if (isForward) {
      m_driveTrainSubsystem.setForward(true);
      m_dashboard.setCamView(Cameras.FORWARDS);
    } else {
      m_driveTrainSubsystem.setForward(false);
      m_dashboard.setCamView(Cameras.REVERSE);
    }
  }

  //Change 0.9 to 0.5 in constants
  public static boolean isRightDriverTriggerPressed() {
    return m_driveController.getRightTriggerAxis() > 0.9;
  }

  public static boolean isLeftDriverTriggerPressed() {
    return m_driveController.getLeftTriggerAxis() > 0.9;
  }

  public static boolean isLeftOpTriggerPressed() {
    return m_opController.getLeftTriggerAxis() > 0.9;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void simulationInit() {
    m_transitionSubsystem.simulationInit();
  }
}
