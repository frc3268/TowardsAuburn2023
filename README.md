# TowardsAuburn2023
I'm hooked on a feelin'  
I'm high on believin'  

Current "Custom" Components:  
- SwerveModule.Kt. Defines and configures a swerve module(1 wheel each) based on params provided in constants. Includes a public setSpeeds() method used to control.  
- DriveTrain.Kt. Defines and configures a swerve drivetrain with 4 swerve modules. includes several public methods to use drivetrain, cheif among them being drive(), which takes in a Translation2d and sets motors accordingly. of note is that the main drive method is FIELD oriented.    

What's Next:  
- Balancing. May need to be its own file to sperate from drivetrain, called in drivetrain nevertheless. Need to add robot openloop mode to swervemodules for this.  
