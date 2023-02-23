
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Modified by Montclair Robotics Team 555

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.framework.RobotState;
import frc.robot.framework.frc.AutoCommands;
import frc.robot.framework.frc.commands.CommandRobot;
import frc.robot.framework.frc.commands.RobotContainer;
import frc.robot.framework.frc.commands.triggers.AnalogTrigger;
import frc.robot.framework.frc.controllers.GameController;
import frc.robot.framework.frc.controllers.GameController.DPad;
import frc.robot.framework.frc.vendors.rev.BlinkinLEDDriver;
import frc.robot.framework.math.MathUtils;
import frc.robot.managers.NavxManager;
import frc.robot.managers.VisionManager;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;
import static frc.robot.framework.frc.commands.Commands.*;
import static frc.robot.framework.frc.controllers.GameController.Axis.*;
import static frc.robot.framework.frc.controllers.GameController.Button.*;
import static frc.robot.framework.frc.vendors.rev.BlinkinLEDMode.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.*;
import static edu.wpi.first.wpilibj2.command.CommandBase.*;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public final class RapidReact extends RobotContainer 
{
    ////////////////////////////////
    // CONTROLLERS
    ////////////////////////////////
    public static final GameController driverController = GameController.from(DRIVER_CONTROLLER_TYPE,
            DRIVER_CONTROLLER_PORT);
    public static final GameController operatorController = GameController.from(OPERATOR_CONTROLLER_TYPE,
            OPERATOR_CONTROLLER_PORT);

    ////////////////////////////////
    // SUBSYSTEMS / MANAGERS
    ////////////////////////////////
    public static final Drivetrain drivetrain = new Drivetrain();

    public static final VisionManager vision = new VisionManager();
    public static final NavxManager navx = new NavxManager(new AHRS());
    
    /*
    public final BlinkinLEDDriver blinkinLEDDriver = new BlinkinLEDDriver(BLINKIN_LED_DRIVER_PORT, C1_BREATH_SLOW, DISABLED);

    public final Ultrasonic ultrasonicLeft = new Ultrasonic(LEFT_ULTRASONIC_SENSOR_PING_PORT, LEFT_ULTRASONIC_SENSOR_ECHO_PORT);
    public final Ultrasonic ultrasonicRight = new Ultrasonic(RIGHT_ULTRASONIC_SENSOR_PING_PORT, RIGHT_ULTRASONIC_SENSOR_ECHO_PORT);
    //*/

    ////////////////////////////////
    // INITIALIZATION
    ////////////////////////////////
    @Override
    public void initialize()
    {
        // Smart dashboard
        
        // Max speed command
        driverController.getButton(A_CROSS)
            .whenActive(drivetrain::nextDriveSpeed);
                 
        // Ease control command
        driverController.getAxis(RIGHT_TRIGGER)
            .whenGreaterThan(0.5)
            .whenActive(() -> {
                drivetrain.setProfiler(NOTHING_PROFILER);
                drivetrain.killMomentum();
            })
            .whenInactive(() -> drivetrain.setProfiler(DRIVE_PROFILER));

        // Drive command
        drivetrain.setDefaultCommand(
            run(() -> 
            {
                if(!DriverStation.isTeleop())
                {
                    return;
                }

                drivetrain.set(
                    -driverController.getAxisValue(LEFT_Y), 
                    driverController.getAxisValue(RIGHT_X)
                );
                //System.out.println(navxTracker.getAngularVelocity());
            }, drivetrain)
        );

        // Turn reverse
        /*
        driverController.getAxis(RIGHT_TRIGGER).whenGreaterThan(0.5)
            .whenActive(drivetrain::startReverseTurning)
            .whenInactive(drivetrain::stopReverseTurning);
        */

        // Turn commands
        driverController.getDPad(DPad.RIGHT)
            .toggleWhenActive(RapidReactCommands.turn(90));
        driverController.getDPad(DPad.LEFT)
            .toggleWhenActive(RapidReactCommands.turn(-90));
        driverController.getDPad(DPad.UP)
            .toggleWhenActive(RapidReactCommands.turn(-180));
        driverController.getDPad(DPad.DOWN)
            .toggleWhenActive(RapidReactCommands.turn(180));

        

        // PID straight angle command: TELEOP ONLY
        driverController.getAxis(RIGHT_X).abs()
            .whenLessThan(ANGLE_PID_DEADBAND)
            .and(
                AnalogTrigger.from(navx::getAngularVelocity).abs()
                    .whenLessThan(ANGLE_VELOCITY_DEADBAND)
            )
            .and(new Trigger(drivetrain::isTargetingAnAngle).negate())
            .and(new Trigger(drivetrain::isStraightPidding).negate())
            .and(new Trigger(DriverStation::isTeleop))
            .whenActive(sequence(
                instant(drivetrain::startStraightPidding),
                waitUntil(() ->
                    Math.abs(driverController.getAxisValue(RIGHT_X)) >= ANGLE_PID_DEADBAND
                    || drivetrain.isTargetingAnAngle()
                ),
                instant(drivetrain::stopStraightPidding)
            ));

        AutoCommands.add("Main", () -> Commands.none());

        AutoCommands.setDefaultAutoCommand("Main");
    }
}

// drive forward 5 feet :P
// hello, my name is my name. I am my name. I am 10 years old.
// my favorite name is my favorite name. I eat food and my favorite name.
// Caitie is my favorite name, shoots is my favorite name, 
// and my favorite name is my favorite name.
// Lila is my least favorite name. I eat food and my favorite bucket of ballMover.
// Open Source Software 
// Update subsystems speed with subsystems smoother my favorite name
// Cesca is my least favorite name Cesca root directory of this project.
// Who is my least favorite? I think that is my favorite name. What about the name of the name of the name of the name of the name of the name of the?
// I think that goes backward. 86.
// Max speed command.
// Max is command least favorite name Max root directory of this project.
// Mert wasCancelled.
// Josh is be least favorite.
// Rohan is not a favorite,

// I think that science is a cool thing
// TODO
// My favorite type of science is my favorite type of science that 
// is my favorite type of science that is my favorite type of science 
// Serena is my least favorite name Serena root directory of this project            

// Modified by Montclair Robotics Team 555 per feet

// I think that the Montclair Robotics Team is a goes 
// In favorite, i this this the Robotics is my favorite   

// I think that the Montclair Robotics Team is my favorite type of science that is my favorite type of science that is my favorite type of science that is my favorite type of science that is my favorite type of science that is 
