// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses the drive subsystem.
 */
public class DefaultDriveCommand extends LoggingCommandBase {

    private final DriveSubsystem             driveSubsystem;
    private final OperatorInput              operatorInput;
    private final SendableChooser<DriveMode> driveModeChooser;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, SendableChooser<DriveMode> driveModeChooser,
        DriveSubsystem driveSubsystem) {

        this.operatorInput    = operatorInput;
        this.driveModeChooser = driveModeChooser;
        this.driveSubsystem   = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        DriveMode driveMode = driveModeChooser.getSelected();

        boolean   boost     = operatorInput.getBoost();

        switch (driveMode) {

        case SINGLE_STICK_ARCADE:
            // DNE
        case DUAL_STICK_ARCADE:
        default:

            double speed = operatorInput.getSpeed(driveMode);
            double turn = operatorInput.getTurn(driveMode);
            setMotorSpeedsArcade(speed, turn, boost);
            break;

        case TANK:

            double leftSpeed = operatorInput.getLeftSpeed();
            double rightSpeed = operatorInput.getRightSpeed();
            if (boost) {
                driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
            }
            else {
                // If not in boost mode, then divide the motors speeds in half
                driveSubsystem.setMotorSpeeds(leftSpeed / 2.0, rightSpeed / 2.0);
            }
            break;
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

    private void setMotorSpeedsArcade(double speed, double turn, boolean boost) {

        double maxSpeed = 1.0;

        if (!boost) {
            speed    /= 2.0;
            turn     /= 2.0;
            maxSpeed /= 2.0;
        }

        // The basic algorithm for arcade is to add the turn and the speed

        double leftSpeed  = speed + turn;
        double rightSpeed = speed - turn;

        // If the speed + turn exceeds the max speed, then keep the differential
        // and reduce the speed of the other motor appropriately

        if (Math.abs(leftSpeed) > maxSpeed || Math.abs(rightSpeed) > maxSpeed) {

            if (Math.abs(leftSpeed) > maxSpeed) {

                if (leftSpeed > 0) {
                    leftSpeed = maxSpeed;
                }
                else {
                    leftSpeed = -maxSpeed;
                }
                rightSpeed = leftSpeed - turn;

            }
            else {

                if (rightSpeed > 0) {
                    rightSpeed = maxSpeed;
                }
                else {
                    rightSpeed = -maxSpeed;
                }

                leftSpeed = rightSpeed + turn;
            }
        }

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }


}