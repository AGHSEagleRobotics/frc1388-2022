// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DashboardConstants.Cameras;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DashboardConstants;

/** Add your docs here. */
public class Dashboard {
    private ShuffleboardTab m_shuffleboardTab;

   // private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();

    //FIXME CHANGE THESE to constants? 
    public ComplexWidget m_complexWidgetAuton;
    public ComplexWidget m_complexWidgetPosition;
    private final int autonChooserWidth = 8;
    private final int autonChooserHeight = 2;
    private final int autonChooserColumnIndex = 0;
    private final int autonChooserRowIndex = 0;

    // front reverse & ball cams
    private final UsbCamera m_frontCamera = CameraServer.startAutomaticCapture(DashboardConstants.FRONT_CAMERA_PORT);
    private final UsbCamera m_reverseCamera = CameraServer.startAutomaticCapture(DashboardConstants.REVERSE_CAMERA_PORT);
    private final UsbCamera m_ballCamera = CameraServer.startAutomaticCapture(DashboardConstants.BALL_CAMERA_PORT);

    private final VideoSink m_driveVideoSink = CameraServer.getServer();
    private final VideoSink m_ballVideoSink  = CameraServer.getServer();

    public ComplexWidget m_complexWidgetCam;

    //TODO put enum back in here (enum's existance subject to debate)
    private Cameras m_currentCam = Cameras.FORWARDS;

    public Dashboard() { // constructer
        setCamView(Cameras.FORWARDS);
        shuffleboardSetUp();
    } // end constructer

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

        LEAVETARMAC ("Leave Tarmac"), //FIXME change these distances (or delete)
        MOVEPICKUPSHOOT2 ("MOVESHOOT2"),
        MOVESHOOT1 ("MOVESHOOT1"),
        LOWSHOOTMOVE ("LOWSHOOTMOVE"),
        DONOTHING ("Does nothing");

        public static final Objective Default = MOVESHOOT1;

        private String name;

        private Objective (String m_name) {
            m_name = name;
        }
        public String getName(){
            return name;
        }

    }

    // public Position getPosition() {
    //     return m_autoPosition.getSelected();
    // }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }

    public void shuffleboardSetUp() {
        m_shuffleboardTab =  Shuffleboard.getTab("Competition");
        Shuffleboard.selectTab("Competition");

        // setup camera widgets
        m_complexWidgetCam = m_shuffleboardTab.add("Cams", m_driveVideoSink.getSource())
        .withWidget(BuiltInWidgets.kCameraStream);
        m_ballVideoSink.setSource(m_ballCamera);

        // setup objective chooser
        for (Dashboard.Objective ep: Objective.values()) {
            m_autoObjective.addOption(ep.getName(), ep);
        }

        m_complexWidgetAuton = Shuffleboard.getTab("Competition").add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(autonChooserWidth, autonChooserHeight)
            .withPosition(autonChooserColumnIndex, autonChooserRowIndex);

    //     for (Position ep: Position.values()) {
    //         m_autoPosition.addOption(ep.getName(), ep);
    //     }

    //     m_complexWidgetPosition = Shuffleboard.getTab("Competition").add("AutoPosition", m_autoPosition)
    //     .withWidget(BuiltInWidgets.kSplitButtonChooser);
    
    }

    public void switchCamera() {
        System.out.println("swich camera method");
        switch (m_currentCam) {
            case FORWARDS: m_driveVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case REVERSE: m_driveVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            default: m_driveVideoSink.setSource(m_reverseCamera);
        }
    }

    public void setCamView(Cameras camera) {
        switch (camera) {
            case FORWARDS:
                m_driveVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            case REVERSE:
                m_driveVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case BALL:
                m_driveVideoSink.setSource(m_ballCamera);
                m_currentCam = Cameras.BALL;
                break;
            default: m_driveVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
        }
    }

}
