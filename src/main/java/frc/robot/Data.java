package frc.robot;

import java.lang.reflect.Modifier;
import java.util.Arrays;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.framework.frc.AutoCommands;
import frc.robot.framework.frc.Sendables;

public final class Data 
{
    private Data(){}
    //fucky wucky

    public static void setup()
    {
        mainTab = Shuffleboard.getTab("Main");
        debugTab = Shuffleboard.getTab("Debug");

        setupMainTab();
        setupDebugTab();
    }

    private static void setupMainTab()
    {
        mainTab.add("Alliance", allianceChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 3)
            .withSize(1, 1);

        var values = mainTab.getLayout("Values", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(1, 3);

        values.addNumber("Angular Velocity", RapidReact.navx::getAngularVelocity);
        values.addNumber("Current Max Speed", RapidReact.drivetrain::getMaxOutput);
        values.addString("Current Easing", () -> RapidReact.drivetrain.getProfiler().getName());

        mainTab.addBoolean("PI?", RapidReact.vision::isWorking)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 3)
            .withSize(1, 1);

        mainTab.add("Auto", AutoCommands.chooser())
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0+2, 3)
            .withSize(2, 1);

        /*
        mainTab.add("Cam 1", CameraServer.getVideo("Shooter Vision").getSource())
            .withPosition(1, 0)
            .withSize(3, 2);
        mainTab.add("Vision", CameraServer.getVideo("Vision Output").getSource())
            .withPosition(1+3, 0)
            .withSize(3, 2);
        */
    }
    private static void setupDebugTab()
    {

        var drive = debugTab.getLayout("Drive", BuiltInLayouts.kList)
            .withPosition(0+3+3, 0)
            .withSize(1, 2);

        var turn = debugTab.getLayout("Turn", BuiltInLayouts.kList)
            .withPosition(0+3+3+1, 0)
            .withSize(1, 2);


        var ball = debugTab.getLayout("Ball", BuiltInLayouts.kList)
            .withPosition(0+3+3+1+1, 0)
            .withSize(1, 2);
        
        debugTab.add("Auto", AutoCommands.chooser())
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0+2, 3)
            .withSize(2, 1);
    }

    public static ShuffleboardTab mainTab() {return mainTab;}
    public static ShuffleboardTab debugTab() {return debugTab;}

    private static ShuffleboardTab mainTab;
    private static ShuffleboardTab debugTab;

    private static NetworkTableEntry 
        distanceToTarget,
        angleToTarget,
        turnMode,
        driveMode,
        turnSpeed,
        driveSpeed,
        angleToBall,
        ballArea
    ;

    private static SendableChooser<String> allianceChooser;

    public static double getDistanceKP() {return 0.035;}
    public static double getDistanceKI() {return 0.0;}
    public static double getDistanceKD() {return 0.0078;}
    public static double getDistanceTolerance() {return 2;}

    public static double getAngleKP() {return 0.0091;}
    public static double getAngleKI() {return 0.002;}
    public static double getAngleKD() {return 0.00135;}
    public static double getAngleTolerance() {return 2.0;}
    public static double getAngleIntMax() {return 0.1;}

    public static double getBallKP() {return 0.19;}
    public static double getBallKI() {return 0.025;}
    public static double getBallKD() {return 0.015;}
    public static double getBallTolerance() {return 0.07;}

    public static void setDistanceToTarget(double value) 
    {
        distanceToTarget.setDouble(value);
    }
    public static void setAngleToTarget(double value) 
    {
        angleToTarget.setDouble(value);
    }

    public static void setTurnMode(String value)
    {
        turnMode.setString(value);
    }
    public static void setDriveMode(String value)
    {
        driveMode.setString(value);
    }

    public static void setTurnSpeed(double value)
    {
        turnSpeed.setDouble(value);
    }
    public static void setDriveSpeed(double value)
    {
        driveSpeed.setDouble(value);
    }

    public static void setAngleToBall(double value)
    {
        angleToBall.setDouble(value);
    }
    public static void setBallArea(double value)
    {
        ballArea.setDouble(value);
    }
}
