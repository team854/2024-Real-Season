// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Axis;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.Stick;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses the drive subsystem.
 */
public class DefaultDriveCommand extends Command {

    private final OperatorInput              operatorInput;
    private final DriveSubsystem             driveSubsystem;
    private final SendableChooser<DriveMode> driveModeChooser;

    /**
     * Default Drive command. This command runs when nothing else is running that
     * uses the drive subsystem.
     *
     * @param operatorInput
     * @param driveSubsystem
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem,
        SendableChooser<DriveMode> driveModeChooser) {

        this.operatorInput    = operatorInput;
        this.driveSubsystem   = driveSubsystem;
        this.driveModeChooser = driveModeChooser;

        // Add required subsystems
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        DriveMode driveMode = driveModeChooser.getSelected();

        boolean   boost     = operatorInput.getBoost();

        double    speed     = 0;
        double    turn      = 0;

        switch (driveMode) {

        case SINGLE_STICK_ARCADE:
            speed = operatorInput.getStick(Stick.LEFT, Axis.Y);
            turn = operatorInput.getStick(Stick.LEFT, Axis.X);

            driveSubsystem.setMotorSpeedsArcade(speed, turn, operatorInput.getBoost());

        case DUAL_STICK_ARCADE:
            speed = operatorInput.getStick(Stick.LEFT, Axis.Y);
            turn = operatorInput.getStick(Stick.RIGHT, Axis.X);


        case TANK:

        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void setMotorArcadeSpeeds(double speed, double turn, boolean boost) {

        double maxSpeed = 1.0;

        if (!boost) {
            speed    /= 2.0;
            turn     /= 2.0;
            maxSpeed /= 2.0;
        }

        double leftSpeed  = speed + turn;
        double rightSpeed = speed - turn;

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
