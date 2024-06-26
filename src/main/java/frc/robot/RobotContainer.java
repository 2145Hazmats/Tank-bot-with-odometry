package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private Drivetrain m_Drivetrain = new Drivetrain();
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
    private final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);

    public RobotContainer() {
        // Allows choosing of the drive type trhough SmartDashboard
        SendableChooser<String> m_driveSendableChooser = new SendableChooser<String>();
        m_driveSendableChooser.setDefaultOption("arcadeDrive",  "arcadeDrive");
        m_driveSendableChooser.addOption("tankDrive", "tankDrive");
        m_driveSendableChooser.addOption("curvatureDrive1", "curvatureDrive1");
        m_driveSendableChooser.addOption("curvatureDrive2", "curvatureDrive2");
        SmartDashboard.putData("Drive Type", m_driveSendableChooser);

        m_Drivetrain.setDefaultCommand(m_Drivetrain.drive(
                m_driverController::getLeftY,
                m_driverController::getRightX,
                m_driverController::getRightY,
                m_driveSendableChooser::getSelected
        ));

        configureBindings();
    }

    private void configureBindings() {
        m_driverController.a().onTrue(Commands.runOnce(() -> m_Drivetrain.resetOdometry()));
    }

    public Command getAutonomousCommand() {
        return null;
    }

}
