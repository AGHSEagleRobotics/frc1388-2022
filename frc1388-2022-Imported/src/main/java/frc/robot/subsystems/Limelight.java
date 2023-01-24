// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTableEntry m_tx;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_ta;
  private final NetworkTableEntry m_tv;

  private final NetworkTable m_table;
  /** Creates a new Limelight. */
  public Limelight() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
    m_tv = m_table.getEntry("tv");


  }

  public double getTx() {
    return m_tx.getDouble(0.0);
  }

  public double getTy() {
    return m_ty.getDouble(0.0);
  }

  public double getTa() {
    return m_ta.getDouble(0.0);
  }

  public double getTv() {
    return m_tv.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // PowerDistribution pdh = new PowerDistribution();
    // System.out.println("pdh 0: " + pdh.getCurrent(0) + " pdh 2: " + pdh.getCurrent(2));
    
  }
}
