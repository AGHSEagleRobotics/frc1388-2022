// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(LED.class);


  private PWMSparkMax m_LedArms;
  private PWMSparkMax m_LedBody;
  private boolean m_isLedSet = false;
  // private double m_value = -0.99;
  // private final double m_change = 0.02;

  /** Creates a new LED. */
  public LED(PWMSparkMax LEDControler) {
    m_LedArms = LEDControler;
  } // end constructor

  /**sets the led value.
  //  * @param num clamps from -0.99 to 0.99  */
  // public void setLED(double num) {
  //   MathUtil.clamp(num, -0.99, 0.99);
  //   m_LedArms.set(num);
  // }
  
  /**change led up */
  // public void increaseLED() {
  //   m_value += m_change;
  //   MathUtil.clamp(m_value, -0.99, 0.99);
  //   m_LedArms.set(m_value);
  // }

  /**change led down */
  // public void decreaseLED() {
  //   m_value -= m_change;
  //   MathUtil.clamp(m_value, -0.99, 0.99);
  //   m_LedArms.set(m_value);
  // }

  // public double getValue() {
  //   return m_value;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // log.info("led value: {}", getValue());
    if (DriverStation.isFMSAttached() && !m_isLedSet) {
      m_isLedSet = true;
      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        m_LedArms.set(-0.01);
        m_LedBody.set(0.87);
      } else {
        m_LedArms.set(-0.19);
        m_LedBody.set(-0.83);
      }
    }

  }
}
