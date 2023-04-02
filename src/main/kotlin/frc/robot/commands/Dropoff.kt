package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.RotationalArmSubsystem
import frc.robot.subsystems.GripperSubsystem

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class Dropoff(arm: RotationalArmSubsystem, gripper:GripperSubsystem) : SequentialCommandGroup() {
    /**
     * Creates a new Dropoff.
     */
    init {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(FooCommand(), BarCommand())
        addCommands(
            gripper.setOut().withTimeout(2.0)
        )
    }
}
