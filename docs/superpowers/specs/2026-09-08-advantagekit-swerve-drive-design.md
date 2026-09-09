# AdvantageKit Swerve Drive (Falcon 500 + Analog Encoders + NavX) — Design

## Context

`claude_swerve` currently has no source tree (repo root only has README/LICENSE/.gitignore).
The robot ("2025-Minibot") is a true 4-corner swerve chassis (SDS MK3, L3-equivalent
gearing) built with 8 Falcon 500 motors (4 drive + 4 turn) and 4 analog absolute encoders
(one per turn axis), plus a NavX gyro (Studica AHRS over MXP SPI). No CANivore — all
devices are on the roboRIO CAN bus.

The `TankDrive` branch already drives this same physical hardware, but in a simplified
tank-style mode: the left-side and right-side drive Falcons are paired via CAN `Follower`
so the robot drives like a 2-motor tank, while the 4 turn Falcons are independently
controlled and seeded from analog encoders. That branch is the source of truth for every
real, measured constant we have (CAN IDs, gear ratios, encoder ports/offsets, PID gains).

Several other branches (`dev`, `bruce_drive`, `lohitSwerve`, `lewis_constants`) already
carry the stock Littleton Robotics AdvantageKit CTRE-swerve template (`Drive`, `Module`,
`ModuleIO*`, `GyroIO*`, `PhoenixOdometryThread`, generated `TunerConstants`), but wired to
CANcoders and populated with placeholder/demo geometry (12"x12" module positions, 2" wheel
radius, CANcoder IDs 9-12, "Drivetrain" CANivore bus) — not this robot's real values.

Research.md (this repo) documents the required adaptation: AdvantageKit's IO-layer
abstraction lets the stock TalonFX swerve template be adapted to analog encoders by (1)
keeping an absolute-position field in `ModuleIOInputs`, (2) rewriting the real hardware IO
to instantiate `AnalogEncoder` objects instead of `CANcoder`, and (3) seeding the Falcon
turn motor's internal rotor position from the analog reading at startup instead of using
CANcoder remote/fused feedback.

## Goal

Build a working AdvantageKit swerve `Drive` subsystem, wired for this robot's real
hardware, replacing the stock template's CANcoder-based module IO with the
analog-encoder-seeded approach already proven in `TankDrive`. Teleop joystick drive +
odometry only — no PathPlanner/autonomous in this pass.

## Architecture

Adapt the stock AdvantageKit CTRE-swerve template's file structure and control-loop
design, but decouple it from CTRE's generated `TunerConstants` / `SwerveModuleConstants<>`
machinery (which assumes CANcoders and is normally produced by Tuner X's swerve project
generator). In its place: a hand-written `DriveConstants.java` holding flat constants and a
small `ModuleConstants` record per corner, modeled directly on `TankDrive`'s existing
`DriveConstants.java`.

### Files (new project, copied skeleton + new/adapted `drive` package)

Project skeleton (copied as-is from `TankDrive`, since `claude_swerve` has none):
`build.gradle`, `settings.gradle`, `gradlew`/`gradlew.bat`, `gradle/wrapper/*`,
`.vscode/*`, `.wpilib/*`, `AdvantageKit-License.md`, `WPILib-License.md`,
`vendordeps/{AdvantageKit,Phoenix6,Studica,WPILibNewCommands}.json` (no PathplannerLib —
out of scope).

`frc.robot` (carried over unchanged from `TankDrive`): `Constants.java`, `Main.java`,
`Robot.java`.

`frc.robot.subsystems.drive` (new/rewritten, modeled on the stock AK template):
- `DriveConstants.java` **(new)** — per-corner `ModuleConstants` (drive motor ID, turn
  motor ID, analog encoder channel, encoder zero offset, inversions, drive/turn PID gains)
  for FL/FR/BL/BR, plus shared constants: wheel radius, drive gear ratio (6.122448979591837),
  turn gear ratio (12.8), module X/Y locations for kinematics, theoretical max linear
  speed, CAN bus (rio, no CANivore).
- `GyroIO.java`, `GyroIONavX.java` — ported from the stock template as-is (no CANcoder
  dependency to begin with).
- `ModuleIO.java` — ported as-is; `ModuleIOInputs` keeps `turnAbsolutePosition` sourced
  from the analog encoder.
- `ModuleIOTalonFX.java` **(rewritten)** — no `CANcoder`. Instantiates a WPILib
  `AnalogEncoder` per module using `DriveConstants`' channel/offset. Turn Falcon closes
  the loop on its own rotor sensor (`SensorToMechanismRatio` = turn gear ratio); at
  construction, seeds the Falcon's internal position from the analog encoder's absolute
  reading (converted to rotations), exactly matching `TankDrive`'s `DriveIOTalonFX`
  seeding logic. Drive Falcon config/control mirrors the stock template (voltage/velocity
  control, current limiting) but keyed off `DriveConstants` values instead of
  `SwerveModuleConstants<>`.
- `ModuleIOSim.java` — adapted to take the new `ModuleConstants` type instead of CTRE's
  `SwerveModuleConstants<>`; motor model switched from Kraken X60 to Falcon 500
  (`DCMotor.getFalcon500Foc(1)`).
- `Module.java` — adapted constructor to take `ModuleConstants` instead of
  `SwerveModuleConstants<>`; wheel radius/gear ratio pulled from `DriveConstants`.
- `PhoenixOdometryThread.java` — ported as-is, with `TunerConstants.kCANBus` replaced by
  `DriveConstants.kCANBus` (rio bus, `isNetworkFD() == false`).
- `Drive.java` — ported from the stock template's non-PathPlanner functionality: swerve
  kinematics, `SwerveDrivePoseEstimator`, high-frequency + gyro-fused odometry, alerts for
  disconnected devices, `runVelocity`/`stop`/`stopWithX`, and SysId + wheel-radius
  characterization hooks. All `TunerConstants.*` references replaced with
  `DriveConstants.*`. No `AutoBuilder`/`PathPlannerLogging`/`LocalADStarAK` — out of scope.

`frc.robot.commands`: `DriveCommands.java` — ported from the stock template: field-relative
joystick drive, joystick-drive-at-angle, feedforward + wheel-radius characterization
commands. No PathPlanner-dependent commands.

`frc.robot.RobotContainer.java` **(rewritten)** — real/sim/replay switch instantiates
`Drive` with `GyroIONavX`/`ModuleIOTalonFX` (real), `GyroIO{}`/`ModuleIOSim` (sim), or
no-op IO (replay). Default command: field-relative joystick drive (left stick
translate, right stick X rotate). X button: X-lock. B button: reset gyro heading. A
`LoggedDashboardChooser` exposes the SysId/wheel-radius characterization routines (no
PathPlanner autochooser). `getAutonomousCommand()` returns `null`, matching `TankDrive`.

## Real vs. placeholder data

**Reused verbatim from `TankDrive` (measured/working):**
- Turn CAN IDs: FL=1, FR=3, BR=5, BL=7. Drive CAN IDs: FL=2, FR=4, BR=6, BL=8 (paired
  per corner, inferred from `TankDrive`'s left-lead/follower = FL+BL and right-lead/
  follower = FR+BR grouping, and confirmed by the user).
- Analog encoder ports 0-3 and their zero offsets (`kFLZero`, `kFRZero`, `kBRZero`,
  `kBLZero`).
- Drive gear ratio: 6.122448979591837 (matches SDS MK3 L3-equivalent, confirmed by user).
- Turn gear ratio: 12.8.
- Drive PID/FF gains: kP=0.1, kV=0.124. Turn PID/FF gains: kP=60, kV=0.124.
- Wheel diameter: 4in (2in radius) — confirmed by user.
- Turn motor inversion: Clockwise_Positive (all four).

**Placeholders — explicitly flagged `// TODO: verify on robot` and called out again at
handoff, not to be trusted without on-robot verification:**
- Track width / wheelbase: 11.5in x 11.5in (square), per the user's rough estimate pending
  exact measurement.
- Theoretical max linear speed: derived (not measured) from Falcon free speed ÷ drive
  gear ratio × wheel circumference (~5.5 m/s).
- Per-corner drive motor inversion and encoder-direction sign (not specified in
  `TankDrive`, since it only ran drive motors in tank pairs) — will default to a
  consistent convention (e.g. all `CounterClockwise_Positive`) and flagged as the #1 thing
  to verify first on-robot, since a wrong sign here causes a module to fight itself.

## Error handling

Match the stock template: `Alert` objects for disconnected drive motor / turn motor / turn
encoder per module, and disconnected gyro, surfaced via WPILib's `Alert` (Elastic/Shuffleboard
visible) — no behavior change beyond stopping modules when disabled (already stock).

## Testing plan

- `./gradlew build` for compilation correctness.
- Desktop `simulateJava` using the `ModuleIOSim` path to sanity-check kinematics and the
  joystick drive command in isolation from hardware.
- On-robot verification (not performable by the agent) is required before competition use:
  encoder zero offsets still correct once code changes wheel-angle math, wheel drive
  directions/inversions, true track width/wheelbase, and max speed calibration.

## Out of scope

PathPlanner autonomous, vision integration, any subsystem other than `drive`.
