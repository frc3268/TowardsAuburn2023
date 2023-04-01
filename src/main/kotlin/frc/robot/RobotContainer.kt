package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Joystick

import frc.robot.subsystems.DriveSubsystem
import frc.robot.commands.JoystickDriveCommand
import frc.robot.commands.Autos
import frc.robot.commands.DriveAmountCommand
import frc.robot.commands.TurnAmountCommand
import frc.robot.commands.SetRotAngleCommand
import frc.robot.subsystems.GripperSubsystem
import frc.robot.subsystems.RotationalArmSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private val exampleSubsystem = ExampleSubsystem()

    // controllers
    private val driverController =
        Joystick(Constants.OperatorConstants.kDriverControllerPort)

    /* Driver Buttons */

    // subsystems
    private val drive: DriveSubsystem = DriveSubsystem(Constants.Field.startingPose)
    private val rotation: RotationalArmSubsystem = RotationalArmSubsystem()
    private val gripp: GripperSubsystem = GripperSubsystem()
    private var toggleTank = false

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        // Configure the trigger bindings
        configureBindings()

        //set the default command for the drivetrain using the joysticks and buttons on the controller
        drive.setDefaultCommand(
            JoystickDriveCommand(
                drive,
                { driverController.getX() },
                { driverController.getY() },
                { toggleTank }
            )
        )
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger#Trigger(java.util.function.BooleanSupplier)] constructor with an arbitrary
     * predicate, or via the named factories in
     * [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule ExampleCommand when exampleCondition changes to true
        // Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        Trigger {driverController.getRawButtonPressed(1)}.onTrue(TurnAmountCommand(0.0, drive))
        Trigger {driverController.getRawButtonPressed(2)}.onTrue(DriveAmountCommand(1.0, drive))
        Trigger {driverController.getRawButtonPressed(3)}.onTrue(SetRotAngleCommand(rotation, 45.0))

        /* TODO later change to always activate as long as is pressed -- Weiju */
        Trigger {driverController.getRawButtonPressed(4)}.onTrue(gripp.setIn().withTimeout(1.0))

        Trigger {driverController.getRawButtonPressed(5)}.onTrue(gripp.setOut().withTimeout(1.0))

        Trigger {driverController.getRawButtonPressed(8)}.onTrue(gripp.stopMotor())
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() =
            // Example command
            Autos.basicAuto(drive)
}
//by: vezzhu vayng
//int main(int argc, char** argv){ printf(NULL); }