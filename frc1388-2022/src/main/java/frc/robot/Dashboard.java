// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoMoveConstants;

/** Add your docs here. */
public class Dashboard {

    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private UsbCamera m_cameraColor;

    public enum Position {
        POSITION1 ("POSITION1"),
        POSITION2 ("POSITION2"),
        POSITION3 ("POSITION3"),
        POSITION4 ("POSITION4");

        public static final Position Default = POSITION1;

        private String name;

        private Position (String m_name) {
            name = m_name; 
        }
        public String getName(){
            return name;
        }
    }

    public enum Objective {
        LEAVETARMAC ("Just leave tarmac", 1), //FIXME change distances to fit
        SHOOTBALL1 ("Shoots first ball", 2),
        PICKUPSHOOT2 ("Pick up 2nd ball, shoot both", 3),
        DONOTHING ("Does nothing (POSITION 2)", 4);

        public static final Objective Default = LEAVETARMAC;

        private String name;
        private double distance;

        private Objective (String m_name, double m_distance) {
            m_name = name;
            m_distance = distance;
        }
        public String getName(){
            return name;
        }
        public double getDistance(){
            return distance;
        }

    }

    public Position getPosition() {
        return m_autoPosition.getSelected();
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }

    public Dashboard() {
        colorcamera();
        shuffleboardSetUp();
    }

    private void colorcamera() {
        m_cameraColor = CameraServer.startAutomaticCapture(AutoMoveConstants.USB_CAMERACOLOR);
    }

    public void shuffleboardSetUp(){
    }
}
