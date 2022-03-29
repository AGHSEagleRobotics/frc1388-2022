// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.pattern.AbstractStyleNameConverter.Blue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(LED.class);

  private PWMSparkMax m_LedArms;
  private PWMSparkMax m_LedBody;
  private boolean m_notAtComp = true;
  private boolean m_shooting = false;
  private boolean m_isBlue; // set in perodic

  /** Creates a new LED. */
  public LED(PWMSparkMax bodyLeds, PWMSparkMax armLeds) {
    m_LedBody = bodyLeds;
      m_LedBody.setSafetyEnabled(false);
    m_LedArms = armLeds;
      m_LedArms.setSafetyEnabled(false);
  } // end constructor


  public void ledShoot() {
    m_shooting = true;
  }

  public void ledNormal() {
    m_shooting = false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // m_shooting = true;
    m_isBlue = (DriverStation.getAlliance() == DriverStation.Alliance.Blue);

    if ((DriverStation.isFMSAttached() || m_notAtComp)) {
      if (m_isBlue) {

        m_LedArms.set(LEDConstants.BLUE_LARSON);
        // log.info("blue arms");

        if (m_shooting) {
          m_LedBody.set(LEDConstants.BLUE_FLASH);
        // log.info("blue shoot");
        } else {
          m_LedBody.set(LEDConstants.BLUE_SOLID);
        // log.info("blue solid");
        }

      } else {

        m_LedArms.set(LEDConstants.RED_LARSON);
        // log.info("red arms");

        if (m_shooting) {
          m_LedBody.set(LEDConstants.RED_FLASH);
        // log.info("red shoot");
        } else { 
          m_LedBody.set(LEDConstants.RED_SOLID);
        // log.info("red solid");
        }

      }
    } // end if

    // log.info("\n\n\n" + m_shooting + "\n\n\n");
  
  }
}
