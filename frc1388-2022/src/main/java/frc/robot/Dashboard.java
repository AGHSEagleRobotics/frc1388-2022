// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** Add your docs here. */
public class Dashboard {

    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();

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
    }

    public enum Objective {
        LEAVETARMAC ("Just leave tarmac"),
        SHOOTBALL1 ("Shoots first ball"),
        PICKUPSHOOT2 ("Pick up 2nd ball, shoot both"),
        DONOTHING ("Does nothing");

        public static final Objective Default = LEAVETARMAC;

        private String name;

        private Objective (String m_name) {
            name = m_name;
        }
    }

    public Position getPosition() {
        return m_autoPosition.getSelected();
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
}
