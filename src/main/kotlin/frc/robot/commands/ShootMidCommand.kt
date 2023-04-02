package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.GripperSubsystem
import frc.robot.subsystems.RotationalArmSubsystem

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class ShootMidCommand(gripper: GripperSubsystem, arm: RotationalArmSubsystem)  : SequentialCommandGroup() {
    /**
     * Creates a new ShootMidCommand.
     */
    init {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(FooCommand(), BarCommand())
        addCommands(
            arm.middle(),
            gripper.setOut()
        )
    }
}
