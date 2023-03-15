# TowardsAuburn2023
I'm hooked on a feelin'  
I'm high on believin'  

Current "Custom" Components:  
- SwerveModule.Kt. Defines and configures a swerve module(1 wheel each) based on params provided in constants. Includes a public setSpeeds() method used to control.  
- DriveTrain.Kt. Defines and configures a swerve drivetrain with 4 swerve modules. includes several public methods to use drivetrain, cheif among them being drive(), which takes in a Translation2d and sets motors accordingly.     
- Camerasubsystem.kt. Interacts with the limelight camera and uses photonvision. includes a distancetotarget method and assoc.

## What's Next:
- Add the camera and create a command to drive to the trajectory described by the target.
    - lx = h/Tan(P)
    - ly = lx/Tan(Y)
    - create a translation2d based on those

## Units
**The base units for all measurements in this codebase are meters (length), degrees (angle), and seconds (time).**
To avoid confusion, always use one of the following to specify the unit of a given measurement. These are defined in `lib/Units.kt`.

Name       | Conversion factor
-----------|---------------------
`.meters`  | 1
`.inch`    | 0.0254
`.deg`     | 1
`.rad`     | ~57.2957795  

## Style guide
As much as possible, this code follows the [Android Kotlin Style Guide](https://developer.android.com/kotlin/style-guide), with the following exceptions:
- Wildcard imports are allowed only for
    ```kotlin
    import frc.robot.lib.units.*
    ```
    *Rationale:* Seldom will units be imported individually; they are almost always all needed. Due to the small number of units available, this should not hinder compile times and should actually be beneficial for readability.
- Imports are not ASCII-sorted. *Rationale:* Sorting would take too much time for relatively little gain.

## Credits  
- Team 364, for their BaseFalconSweve code, used in much of our sverwe implementation
- Team 3512, for their SwereBot-2022 code, used to adapt 364's swerve code to fit NEO motors
- The RevRobotics, Kauai Labs, and PhotonVision teams for their vendor libraries
- WPI, for WPILIB
- Brenek Harrison, for the "Kotlin for FRC" extension
