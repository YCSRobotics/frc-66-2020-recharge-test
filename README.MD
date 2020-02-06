# FRC 66 Infinite Recharge
This is the robot code for FRC Team 66 for the 2020 game, Infinite Recharge

Constants are in kHungarian Notation

```
public static final int kIntConstant = 0;
public static final String kStringConstant = "Hello!";
```

Robot functions are split into various classes
- Drivetrain
- Pathfollowing
- Sensordata
- Turret
- Constants
- JoystickOI

The usage of static functions are allowed, assuming the static function is in the relating class. A notable exception to this is SensorData, as this contains gyro specific data.

## Restrictions
- Code should **not** be placed in Robot.java (this excludes initialization and system periodic functions such as ``drivetrain.updatePeriodic()``).
- Do **not** commit non-project files (no cat memes).
- Do **not** modify any of the non-code coonfiguration files(``build.gradle``).

Code is licensed under MIT.