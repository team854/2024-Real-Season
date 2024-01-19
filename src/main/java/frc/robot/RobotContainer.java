// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the robot subsystem are declared.
 * <p>
 * The operatorInput binds buttons to commands.
 */
public class RobotContainer {

    // Declare the operator input class
    private final OperatorInput                operatorInput      = new OperatorInput();

    // Declare all subsystems.
    private final DriveSubsystem               driveSubsystem     = new DriveSubsystem();
    private final SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();

    // AutoPattern stuff
    private final SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();

    // private final SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Set the default commands for all subsystems
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, driveModeChooser, driveSubsystem));

        // Configure the operator input button bindings.
        operatorInput.configureButtonBindings(driveSubsystem);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Pass all subsystems to the auto constructor.
        return new AutonomousCommand(driveSubsystem, autoPatternChooser);
    }
}
