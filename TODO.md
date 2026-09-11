# TODO — Swerve Drive Follow-Up Checklist

This branch (`claude_swerve`) has a complete, working swerve drive
implementation, but a few things were **placeholders** because nobody had
measured the real robot yet, or because they can only be checked once the
code is actually running on hardware. This file lists every one of those
remaining items, in the order you should do them, with step-by-step
instructions. (Completed items are removed from this file as they're
finished, so everything here is still open.)

You don't need to be an expert to do any of this. Just go in order, follow
the steps, and ask a mentor if something doesn't match what's described
here (that usually means something about the real robot is different from
what we assumed, which is exactly the kind of thing this checklist is for).

**After you change any `.java` file, always do the same three things:**
1. Save the file.
2. Run `./gradlew build` in a terminal in the project folder, and make sure
   it says `BUILD SUCCESSFUL` at the end (if it doesn't, you made a typo —
   read the error message, it usually tells you the exact line).
3. Deploy the code to the robot (in VS Code: press `Shift+F5`, or use the
   WPILib icon in the top right → "Deploy Robot Code").

---

## Priority 1 — Do this before you drive the robot for the first time

Do these with the robot **up on blocks or a stand, wheels off the ground**,
so that if a wheel spins the wrong way, it can't run into anything or hurt
anyone.

### 1. Check that every wheel drives the correct direction

**Why:** This robot's code assumes that the left-side wheels (front-left
and back-left) spin one way when driving "forward," and the right-side
wheels (front-right and back-right) spin the opposite way — because that's
how a similar test rig (the `TankDrive` branch) was wired. But this is the
**first time** all four wheels have ever been driven as four *independent*
swerve modules instead of as a simple left/right pair, so this assumption
has never actually been tested. If it's wrong, a wheel will fight against
the others when you try to drive straight.

**Steps:**
1. With the robot on blocks and enabled in Teleop mode, gently push the
   left joystick forward just a little bit (a small amount of stick input
   is enough — you don't need full speed for this check).
2. Watch all four wheels. Each wheel's drive motor should spin in the
   direction that would push the robot forward if the wheels were touching
   the ground (imagine the robot driving away from you).
3. **If all four wheels look correct:** great, you're done with this step,
   no code changes needed.
4. **If one entire side (both wheels on the left, or both wheels on the
   right) spins backward:** open
   `src/main/java/frc/robot/subsystems/drive/DriveConstants.java` and find
   these two lines (near the middle of the file):
   ```java
   private static final boolean kLeftDriveInverted = false;
   private static final boolean kRightDriveInverted = true;
   ```
   Flip the `true`/`false` for whichever side was backward (for example, if
   the left side was backward, change `kLeftDriveInverted = false;` to
   `kLeftDriveInverted = true;`). Save, rebuild, redeploy, and test again.
5. **If only ONE wheel (not a whole side) spins backward, or a wheel spins
   in a weird diagonal way:** stop and get a mentor. That means a wire is
   probably plugged into the wrong motor controller, or a motor's CAN ID
   doesn't match what the code expects (`FrontLeft` drive = CAN ID 2, turn
   = CAN ID 1; `FrontRight` drive = 4, turn = 3; `BackLeft` drive = 8, turn
   = 7; `BackRight` drive = 6, turn = 5 — these are set in
   `DriveConstants.java`). This is a wiring/hardware problem, not something
   you fix by flipping a `true`/`false` in the code.

---

## Priority 2 — Do this once the robot is driving safely, to make it drive its best

These use built-in tools that are already wired into the code — you don't
need to write any new code for these, you just need to run them from the
driver station.

### 2. Run the Wheel Radius Characterization routine

**Why:** The code assumes your wheels are exactly 2 inches in radius
(4 inches across). In reality, wheels wear down and are never perfectly
the size they were designed to be, which makes the robot's odometry
(its sense of "where am I on the field") drift over time. This routine
measures your *actual* wheel radius by spinning the robot in a circle and
comparing what the wheels report to what the gyroscope reports.

**Steps:**
1. Put the robot on the ground with plenty of open space around it — it
   will spin in place during this test.
2. Open the driver station software (Shuffleboard, Elastic, or whatever
   your team uses) and find the **Autonomous** mode chooser. It should have
   an option called **"Drive Wheel Radius Characterization"**.
3. Select it, then enable the robot in **Autonomous** mode. It will slowly
   spin in place and print results.
4. Look at the console/log output for a line like:
   ```
   Wheel Radius: 0.0508 meters, 2.000 inches
   ```
5. If the printed number is meaningfully different from 2.000 inches (say,
   more than a hundredth of an inch off), open `DriveConstants.java` and
   update this line:
   ```java
   public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
   ```
   Replace `2.0` with the measured value in inches.
6. Save, rebuild, redeploy.

### 3. Run the Feedforward Characterization routine and set a safe max speed

**Why:** The code currently calculates the robot's theoretical top speed
using a math formula (motor speed ÷ gearing × wheel size), which assumes
a perfect motor with no friction, no battery voltage sag, and a perfectly
smooth floor. Real robots never actually reach their theoretical top
speed — if you tell the robot to drive at a speed it physically can't
reach, its steering and speed control will behave badly (the technical
term is "saturating"). Racing/FRC teams typically aim for about 80-90% of
the theoretical number as a safer, more honest max speed.

**Steps:**
1. Put the robot on the ground with a lot of open space in front of it —
   this test drives it in a straight line, speeding up gradually.
2. In the driver station's Autonomous mode chooser, select **"Drive
   Simple FF Characterization"**.
3. Enable the robot in **Autonomous** mode and let it run (it will
   accelerate slowly, then stop on its own).
4. Look at the console/log output for two printed numbers, `kS` and `kV`.
   You don't need to use these directly in `DriveConstants.java` right now
   (they're mainly useful if you want more advanced velocity control
   later) — but this test confirms the drivetrain can actually reach a
   reasonable top speed.
5. Open `DriveConstants.java` and find this block:
   ```java
   public static final double kMaxLinearSpeedMetersPerSec =
       DCMotor.getFalcon500(1).freeSpeedRadPerSec / kDriveGearRatio * kWheelRadiusMeters;
   ```
   This is the "perfect world" theoretical number. Multiply it by a safety
   factor (0.85 is a reasonable starting point — ask a mentor if you want
   to tune this further) by changing it to:
   ```java
   public static final double kMaxLinearSpeedMetersPerSec =
       0.85 * (DCMotor.getFalcon500(1).freeSpeedRadPerSec / kDriveGearRatio * kWheelRadiusMeters);
   ```
6. Also delete the `// TODO: verify on robot` comment directly above this
   block, since you've now addressed it.
7. Save, rebuild, redeploy, and confirm the robot still drives smoothly at
   full stick.

**Note:** This is separate from the dashboard "Speed Limit" chooser
(30/40/50/60/70%, default 50%) added in `RobotContainer.java` for safe
outreach/demo driving around the public — that chooser scales *this*
`kMaxLinearSpeedMetersPerSec` value down further, without needing a
redeploy. Set `kMaxLinearSpeedMetersPerSec` to the robot's real honest top
speed here; use the dashboard chooser at event time to pick how much of
that speed is safe to expose to a driver that day.

### 4. Double-check the turn (steering) zero positions

**Why:** Each swerve module has a specific "zero" angle number in the code
(one per wheel) that tells the robot "this is what straight-ahead looks
like" for that wheel's steering sensor. These four numbers were copied
from an earlier version of the code (`TankDrive`) that only ever drove
the wheels as a simple tank-style setup, not as full independent swerve
modules. They are very likely still correct, but this is the first time
they've been used for real swerve steering, so it's worth double-checking.

**Steps:**
1. With the robot on blocks, enable Teleop and don't touch the joystick at
   all — the robot should hold all four wheels pointing straight forward
   (this happens automatically; there's no separate "zero" command to
   run).
2. Look at each wheel. If a wheel is not pointing straight forward, note
   which one (front-left, front-right, back-left, or back-right).
3. If a wheel is off, get a mentor to help you find its current angle
   reading (there are tools in AdvantageScope, the log-viewing app that
   comes with this project, to see live sensor values) and update that
   wheel's "zero" number in `DriveConstants.java`. The four numbers to look
   at are the third-to-last argument in each of these four lines:
   ```java
   public static final ModuleConstants kFrontLeft =
       new ModuleConstants(2, 1, kLeftDriveInverted, 0, 5.710, kHalfWheelBaseMeters, kHalfTrackWidthMeters);
   public static final ModuleConstants kFrontRight =
       new ModuleConstants(4, 3, kRightDriveInverted, 1, 3.735, kHalfWheelBaseMeters, -kHalfTrackWidthMeters);
   public static final ModuleConstants kBackLeft =
       new ModuleConstants(8, 7, kLeftDriveInverted, 3, 5.8, -kHalfWheelBaseMeters, kHalfTrackWidthMeters);
   public static final ModuleConstants kBackRight =
       new ModuleConstants(6, 5, kRightDriveInverted, 2, 2.169, -kHalfWheelBaseMeters, -kHalfTrackWidthMeters);
   ```
   (`5.710` for front-left, `3.735` for front-right, `5.8` for back-left,
   `2.169` for back-right.) This step is fiddly — don't guess, work with a
   mentor the first time you do it.

---

## Priority 3 — Optional improvements (nice to have, not urgent)

These don't need to be done before competing, but a mentor or a more
experienced teammate may want to tackle them eventually.

### 5. Protect against a rare number-overflow bug if a CANivore is ever added

**What it means:** `GyroIONavX.java` has a line that converts the robot's
sensor-update-rate number into a `byte` (a very small type of number that
can only hold values from -128 to 127):
```java
private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
```
Right now `Drive.ODOMETRY_FREQUENCY` is `100`, which fits fine. But if this
robot ever gets a CANivore (an upgraded CAN networking device some FRC
teams add), the code elsewhere automatically raises that number to `250`,
which does **not** fit in a `byte` and would silently wrap around to a
negative number, breaking the gyro update rate. This robot doesn't use a
CANivore today, so it's not an active bug — just something to fix if a
CANivore is ever added. A mentor can help clamp the value or add a
comment/assertion so it fails loudly instead of silently if this ever
happens.

### 6. Note on sensor "disconnected" detection timing

**What it means:** When the drive or turn motor's connection is checked
each cycle (`driveConnectedDebounce`/`turnConnectedDebounce` in
`ModuleIOTalonFX.java`), the code waits half a second of good readings
before reporting "connected" again after a disconnect, but reports
"disconnected" instantly on the very first bad reading. This matches the
standard AdvantageKit template's behavior and is intentional, not a bug —
just worth knowing about if you ever see a disconnected-motor warning
flash briefly on startup before settling down.
