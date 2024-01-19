package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeading extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         targetHeading;
    private double         currentHeading;
    private double         degreeDiff;
    private double         speed;
    private boolean        brakeAtEnd;

    private double         pTerm;
    private double         iTerm;
    private double         dTerm;
    private double         previousError;
    private double         currentError;
    private double         leftSpeed;
    private double         rightSpeed;



    public TurnToHeading(double speed, double targetHeading, boolean brakeAtEnd, DriveSubsystem driveSubsystem) {
        this.speed          = speed;
        this.brakeAtEnd     = brakeAtEnd;
        this.targetHeading  = targetHeading;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetHeading;
        logCommandStart(commandParms);

        currentHeading = driveSubsystem.getYaw();

        double cwDist = targetHeading - currentHeading;
        if (cwDist < 0) {
            cwDist = cwDist + 360;
        }
        double ccwDist = currentHeading - targetHeading;
        if (ccwDist < 0) {
            ccwDist = ccwDist + 360;
        }

        degreeDiff = cwDist;
    }

    @Override
    public void execute() {

        // executes every 20ms

        currentHeading = driveSubsystem.getYaw();

        double cwDist = targetHeading - currentHeading;
        if (cwDist < 0) {
            cwDist = cwDist + 360;
        }
        double ccwDist = currentHeading - targetHeading;
        if (ccwDist < 0) {
            ccwDist = ccwDist + 360;
        }


        if (cwDist <= ccwDist) {
            currentError = cwDist;
        }
        else {
            currentError = ccwDist;
        }

        double diffError = currentError - previousError;
        previousError  = currentError;

        pTerm          = DriveConstants.TURN_TO_HEADING_PID_KP * currentError;
        iTerm         += DriveConstants.TURN_TO_HEADING_PID_KI * currentError;
        dTerm         += DriveConstants.TURN_TO_HEADING_PID_KD * diffError;


        double errorSignal = pTerm + iTerm + dTerm;


        if (cwDist <= ccwDist) {
            leftSpeed  = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);
            rightSpeed = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
        }
        else {
            leftSpeed  = Math.min(Math.max(speed - errorSignal, -1.0), 1.0);
            rightSpeed = Math.min(Math.max(speed + errorSignal, -1.0), 1.0);
        }


        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        if (degreeDiff <= Constants.DriveConstants.HEADING_ERROR_BUFFER) {
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }

        logCommandEnd(interrupted);
    }

}
