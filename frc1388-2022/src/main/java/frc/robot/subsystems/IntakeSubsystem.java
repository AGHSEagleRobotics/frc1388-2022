// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_intakeArmMotor;
  private final CANSparkMax m_intakeWheelSpin;
  private final Encoder m_intakeArmEncoder;
  private final DigitalInput m_intakeLimitUp;
  private boolean m_isEncoderReset = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(WPI_TalonSRX intakeArmMotor, CANSparkMax intakeWheelSpin, DigitalInput intakeLimitUp, Encoder intakeArmEncoder) {
    m_intakeArmMotor = intakeArmMotor;
    m_intakeWheelSpin = intakeWheelSpin;
    m_intakeLimitUp = intakeLimitUp;
    m_intakeArmEncoder = intakeArmEncoder;

    m_intakeArmMotor.configFactoryDefault();
    m_intakeArmMotor.setInverted(true);
    m_intakeArmMotor.setNeutralMode(NeutralMode.Brake);

    m_intakeWheelSpin.restoreFactoryDefaults();
    m_intakeWheelSpin.setIdleMode(IdleMode.kCoast);
    m_intakeWheelSpin.setInverted(true);

    m_intakeArmEncoder.reset();
   }

   public void setIntakeWheelSpin(double speed){
     m_intakeWheelSpin.set(speed);
   }

   public void setIntakeArmMotor(double speed){
     /* If the speed is positive and the encoder is less than max (deploying) or
     * If the speed is negative and limit is not tripped (retracting)
     * then set the speed for the intake  (allow intake to run)
     */
    //ARM DOWN ENCODER COUNT is the rotations of the intake times the encoder counts per rev
    //ARM DOWN ENCODER COUNT functions as a max value. 
    boolean intakeLimitUp = m_intakeLimitUp.get();
    if (    (   m_intakeArmEncoder.get() < INTAKE_ARM_DOWN_ENCODER_COUNT 
             && speed > 0 
             && m_isEncoderReset) 
        ||  (   speed < 0 
             && !intakeLimitUp)
        || speed == 0)
    {
      m_intakeArmMotor.set(speed); 
    }
    else
    {
      m_intakeArmMotor.set(0);
    } //else setmotor raise arm until hit limit switch
   }

   //We need to allow the encoder to be reset when the limit switch is triggered - what's going to trigger the arm to move up? Teleop? Auto?
   //Turn off the motor and reset encoder when limit switch is hit (periodic)
   //Turn off motor when INTAKE_ARM_DOWN_ENCODER_COUNT has been reached

  //
  @Override
  public void periodic() {
    boolean intakeLimitUp = m_intakeLimitUp.get();
    int intakeArmEncoder = m_intakeArmEncoder.get();
    double intakeArmMotorSpeed = m_intakeArmMotor.get();

    // System.out.println(intakeArmEncoder);

    //reset encoder when limit switch pressed
    if (intakeLimitUp && !m_isEncoderReset) {
      m_intakeArmEncoder.reset();
      m_isEncoderReset = true;
    }
    // stop arm motor when arm down encoder count constant is reached
    if ( (intakeArmMotorSpeed > 0) && (intakeArmEncoder > INTAKE_ARM_DOWN_ENCODER_COUNT) ) {
      m_intakeArmMotor.set(0);
    }
    // stop arm motor when retracting limit switch is reached
    if (intakeArmMotorSpeed < 0 && intakeLimitUp) {
      m_intakeArmMotor.set(0);
    }
    // This method will be called once per scheduler run

    // System.out.println("encoder: " + m_intakeArmEncoder.get());

  }

  public boolean isLimitUpReached() {
    return ( (m_intakeLimitUp.get()) );
  }

  public boolean isLimitDownReached() {
    return ( m_intakeArmEncoder.get() > INTAKE_ARM_DOWN_ENCODER_COUNT);
  }

  public boolean isEncoderReset() {
    return (m_isEncoderReset);
  }

  public boolean isCloseToUpLimit() {
    return ((m_intakeArmEncoder.get() < INTAKE_ARM_NEAR_UP_ENCODER_COUNT) && m_isEncoderReset);
  }
}
