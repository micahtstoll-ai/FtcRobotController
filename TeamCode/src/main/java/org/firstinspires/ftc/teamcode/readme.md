## TeamCode module

Team-authored robot code. The FtcRobotController module ships the SDK and sample
OpModes; anything under this module is ours.

### Package layout

    org.firstinspires.ftc.teamcode
      hardware/       Single source of truth for hardware configuration.
                      Any change to a hardware-map name or wiring convention
                      (motor direction, odometry pod offset, encoder direction)
                      belongs in RobotHardware and nowhere else.

      subsystems/     Loop-time behavior wrapped around hardware.
                      A subsystem owns its motors, exposes intent-level methods
                      to callers (drive, setDesiredRpm), and hides state that
                      would otherwise leak into an OpMode's main loop.

      opmodes/        The classes the Driver Station lists. Compose subsystems;
                      keep the loop small enough that reading it end-to-end
                      tells you what the driver will feel.

      pedroPathing/   Pedro Pathing configuration and paths. Pedro's classes
                      expect this exact package name; do not move them.

### Rules

- OpModes must not construct `hardwareMap.get(...)` calls directly for anything
  that has a subsystem. If a piece of hardware has no subsystem yet, either add
  one or add a factory to `RobotHardware`.

- Hardware-map string names live only in `RobotHardware`. If you see a literal
  `"leftFront"` outside that class, that is a bug.

- Physical wiring conventions (motor direction, pod direction, pod offset) live
  only in `RobotHardware`. Pedro's `Constants.java` reads them from there;
  OpModes read them via subsystems.

- New OpMode names use the pattern `"TeleOp: X"` or `"Auto: X"` for Driver
  Station grouping.

### Adding a new subsystem

1. Add hardware-map name and any wiring constants to `RobotHardware`.
2. Add a factory method to `RobotHardware` that returns the configured motor(s).
3. Write the subsystem in `subsystems/`; construct it with a `HardwareMap` and
   let `RobotHardware` do the wiring.
4. OpModes construct the subsystem and call intent-level methods.
