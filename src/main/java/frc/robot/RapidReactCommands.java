package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import java.util.function.DoubleSupplier;

import static frc.robot.RapidReact.*;
import static frc.robot.framework.frc.commands.Commands.*;

public final class RapidReactCommands
{
    private RapidReactCommands() {}

    public static Command turn(double degrees)
    {
        return turn(() -> degrees);
    }
    public static Command turn(DoubleSupplier degrees)
    {
        return sequence(
            race(
                waitFor(3.5 + Math.abs(degrees.getAsDouble() / 180.0)), // fail safe in event of lock up
                sequence(
                    instant(() -> drivetrain.setTargetAngle(degrees.getAsDouble())),
                    waitUntil(drivetrain::reachedTargetAngle)
                )
            ),
            instant(drivetrain::releaseAngleTarget)
        );
    }

}
