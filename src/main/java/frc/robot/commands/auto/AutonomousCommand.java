// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.commands.drive.MeasuredStraightDriveCommand;
import frc.robot.commands.drive.TimedDriveCommand;
import frc.robot.commands.drive.TimedStraightDriveCommand;
import frc.robot.commands.drive.TurnToHeading;
import frc.robot.subsystems.DriveSubsystem;

public final class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem driveSubsystem,
        SendableChooser<AutoPattern> autoPatternChooser) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoPattern                      autoPattern = autoPatternChooser.getSelected();

        Optional<DriverStation.Alliance> alliance    = DriverStation.getAlliance();

        StringBuilder                    sb          = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Auto Pattern  : ").append(autoPattern);
        sb.append("\n   Alliance      : ").append(alliance);

        System.out.println(sb.toString());

        // If any inputs are null, then there was some kind of error.
        if (autoPattern == null) {
            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
            return;
        }
        /*
         * else if (alliance == Alliance.Invalid) {
         * System.out.println("*** ERROR *** Invalid alliance");
         * return;
         * }
         */
        // the alliance enum doesnt have and Invalid element for some reason and we dont have
        // writing perms

        /*
         * Compose the appropriate auto commands
         */
        switch (autoPattern) {

        case DO_NOTHING:
        default:
            return;

        case DRIVE_FORWARD:
            // Drive forward for 2 seconds
            addCommands(new TimedDriveCommand(2000, 0.5, 0.5, true, driveSubsystem));

        case DRIVE_FORWARD_PID_TIMED:
            // Drive forward for 1 second
            addCommands(new TimedStraightDriveCommand(1000, 0.5, true, 0, driveSubsystem));


        case DRIVE_FORWARD_PID_MEASURED:
            // Drive forward for 5 meters
            addCommands(new MeasuredStraightDriveCommand(500, 0.25, true, driveSubsystem));

        case TURN_TO_HEADING:
            // Turn to a 30 degree heading
            addCommands(new TurnToHeading(0.25, 30, true, driveSubsystem));

        }
    }


}