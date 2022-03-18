// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.type.ResolvedType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DashboardConstants.Cameras;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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

    //Change these to constants?
    public ComplexWidget m_complexWidgetObjective;
    public ComplexWidget m_complexWidgetPosition;
    private final int autonChooserWidth = 9;
    private final int autonChooserHeight = 3;
    private final int autonChooserColumnIndex = 12;
    private final int autonChooserRowIndex = 0;

    // front reverse & ball cams
    private UsbCamera m_frontCamera = CameraServer.startAutomaticCapture(DashboardConstants.FRONT_CAMERA_PORT);
    private UsbCamera m_reverseCamera = CameraServer.startAutomaticCapture(DashboardConstants.REVERSE_CAMERA_PORT);
    private UsbCamera m_ballCamera = CameraServer.startAutomaticCapture(DashboardConstants.BALL_CAMERA_PORT);

    private final VideoSink m_driveVideoSink = CameraServer.getServer();
    private final VideoSink m_ballVideoSink  = CameraServer.getServer();

    private ComplexWidget m_complexWidgetDriveCam;
    private ComplexWidget m_complexWidgetBallCam;

    //TODO put enum back in here (enum's existance subject to debate)
    private Cameras m_currentDriveCam = Cameras.FORWARDS;

    public Dashboard() { // constructer
        // setCamView(Cameras.FORWARDS);
        // setCamView(Cameras.BALL);
        shuffleboardSetUp();
    } // end constructer

    public enum Position {
        POSITION1 ("POSITION1"),
        POSITION2 ("POSITION2"),
        POSITION3 ("POSITION3"),
        POSITION4 ("POSITION4");

        public static final Position Default = POSITION1;

        private String m_name;

        private Position (String name) {
            m_name = name; 
        }
        public String getName(){
            return m_name;
        }
    }

    public enum Objective {

        LEAVETARMAC ("LeaveTarmac"),
        MOVEPICKUPSHOOT2 ("PickUpShoot2"),
        MOVESHOOT1 ("Shoot1Turn"),
        LOWSHOOTMOVE ("LowShoot"),
        DONOTHING ("Nothing");

        public static final Objective Default = MOVESHOOT1;

        private String m_name;

        private Objective (String name) {
            m_name = name;
        }
        public String getName(){
            return m_name;
        }

    }

    // public Position getPosition() {
    //     return m_autoPosition.getSelected();
    // }

    public void shuffleboardSetUp() {
        m_shuffleboardTab =  Shuffleboard.getTab("Competition");
        Shuffleboard.selectTab("Competition");

        
        // setup camera widgets
        m_ballVideoSink.setSource(m_ballCamera);
        m_driveVideoSink.setSource(m_frontCamera);
        
        m_frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_frontCamera.setFPS(20);
        m_frontCamera.setResolution(320, 240);
        m_reverseCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_reverseCamera.setFPS(20);
        m_reverseCamera.setResolution(320, 240);
        m_ballCamera.setFPS(20);
        m_ballCamera.setResolution(40, 30);


        
        m_complexWidgetDriveCam = m_shuffleboardTab.add("Drive", m_driveVideoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(12, 10)
            .withPosition(0, 0);
        // m_complexWidgetBallCam  = m_shuffleboardTab.add("Ball Color", m_driveVideoSink.getSource())
        //     .withWidget(BuiltInWidgets.kCameraStream)
        //     .withSize(9, 7)
        //     .withPosition(12, 3);

        m_shuffleboardTab.addCamera("Ball", "ballcam", "http://roboRIO-1388-FRC.local:1183/?action=stream")
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(9, 7)
            .withPosition(12, 3);


       

        // m_complexWidgetDriveCam = m_shuffleboardTab.add("Cams", m_driveVideoSink.getSource())
        // .withWidget(BuiltInWidgets.kCameraStream);
        // m_ballVideoSink.setSource(m_ballCamera);

        // setup objective chooser
        for (Dashboard.Objective o: Objective.values()) {
            m_autoObjective.addOption(o.getName(), o);
        }
        m_autoObjective.setDefaultOption(Objective.Default.getName(), Objective.Default);

        m_complexWidgetObjective = Shuffleboard.getTab("Competition").add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(autonChooserWidth, autonChooserHeight)
            .withPosition(autonChooserColumnIndex, autonChooserRowIndex);

    //     for (Position ep: Position.values()) {
    //         m_autoPosition.addOption(ep.getName(), ep);
    //     }

    //     m_complexWidgetPosition = Shuffleboard.getTab("Competition").add("AutoPosition", m_autoPosition)
    //     .withWidget(BuiltInWidgets.kSplitButtonChooser);
    
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
    public void switchCamera() {
        System.out.println("swich camera method");
        switch (m_currentDriveCam) {
            case FORWARDS: m_driveVideoSink.setSource(m_reverseCamera);
                m_currentDriveCam = Cameras.REVERSE;
                break;
            case REVERSE: m_driveVideoSink.setSource(m_frontCamera);
                m_currentDriveCam = Cameras.FORWARDS;
                break;
            default: m_driveVideoSink.setSource(m_reverseCamera);
        }
    }

    public void setCamView(Cameras camera) {
        switch (camera) {
            case FORWARDS:
                m_driveVideoSink.setSource(m_frontCamera);
                m_currentDriveCam = Cameras.FORWARDS;
                break;
            case REVERSE:
                m_driveVideoSink.setSource(m_reverseCamera);
                m_currentDriveCam = Cameras.REVERSE;
                break;
            // case BALL:
            //     m_ballVideoSink.setSource(m_ballCamera);
            //     m_currentCam = Cameras.BALL;
            //     break;
            default: m_driveVideoSink.setSource(m_frontCamera);
                m_currentDriveCam = Cameras.FORWARDS;
        }
    }

}
