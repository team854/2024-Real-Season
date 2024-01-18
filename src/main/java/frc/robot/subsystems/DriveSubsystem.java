// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final VictorSPX leftPrimaryMotor   = new VictorSPX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX  leftFollowerMotor  = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final VictorSPX rightPrimaryMotor  = new VictorSPX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX  rightFollowerMotor = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    // Gyro sensor
    private final AHRS      gyroSensorAHRS     = new AHRS();

    // Speed Variables
    private double          leftSpeed          = 0;
    private double          rightSpeed         = 0;


    public DriveSubsystem() {

        // Inverts one sides (Depends on how the gearbox is set up)
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        // Makes it brake if it is not moving
        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);
        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        // Makes one of the motors follow the other
        rightFollowerMotor.follow(rightPrimaryMotor);
        leftFollowerMotor.follow(leftPrimaryMotor);
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    // Stops both motors
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left Motor", leftSpeed);


    }

    // Returns the yaw in degrees
    public double getYaw() {
        double yawAngle = (Math.round(gyroSensorAHRS.getYaw() * 10) / 10.0d) % 360;
        if (yawAngle < 0) {
            yawAngle += 360;
        }
        return yawAngle;
    }

    // Makes sure it is within range
    public double getHeadingError(double targetHeading) {
        double currentHeading = getYaw();
        double error          = currentHeading - targetHeading;

        // ensures that the error signal is not above 180
        if (error > 180) {
            error -= 360;
        }
        // ensures that the error signal is not below -180
        if (error < -180) {
            error += 360;
        }

        return error;
    }

}
