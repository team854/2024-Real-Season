package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Operator Input
 * <p>
 * This class is used to bind buttons to functions.
 * <p>
 * The operator input class should be passed into any command that requires operator input
 */
public class OperatorInput extends SubsystemBase {

    XboxController driverController = new XboxController(OperatorInputConstants.DRIVER_CONTROLLER_PORT);

    /**
     * Use this method to define bindings of buttons to command
     */
    public void configureBindings(DriveSubsystem driveSubsystem) {

        // Configure button bindings to commands by declaring new Triggers
        // Triggers automatically get checked every loop.
    }

    /*
     * Any command where operator input is required will need to get functional instructions from the controller
     */
    public double getDriveSpeed() {
        double stickValue = driverController.getRightY();
        return stickValue;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
