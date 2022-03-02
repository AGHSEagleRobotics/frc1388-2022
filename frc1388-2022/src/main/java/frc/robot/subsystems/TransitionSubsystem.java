// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TransitionSubsystem extends SubsystemBase {
  public static final String TransitionConstants = null;
  /** Creates a new TransitionSubsystem. */
  private final CANSparkMax m_transitionMotor;

  // private static final int PID_IDX = 0; ??

  public TransitionSubsystem(CANSparkMax transitionMotor) {
    m_transitionMotor = transitionMotor;

    m_transitionMotor.restoreFactoryDefaults();
    m_transitionMotor.setIdleMode(IdleMode.kCoast);
  }
  
  //Voltage for simulation
  public void setTransitionSpeed(double speed){
    if (Robot.isSimulation()) {
      m_transitionMotor.setVoltage(speed);
    } else {
      m_transitionMotor.set(speed);
    }
  }

  //For simulation
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_transitionMotor, DCMotor.getNeo550(1));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
