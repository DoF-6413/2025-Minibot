# Drive Subsystem TODO / Bug Log

Tracks issues found in the swerve `Drive` subsystem (branch `bruce_drive`,
based on `origin/drive`) and their fix/verification status. Append new
issues as they're found; update status as fixes are tested on hardware.

Status values: `OPEN` (found, not fixed) · `FIXED-UNTESTED` (code changed,
not yet verified on robot) · `VERIFIED` (confirmed fixed on hardware) ·
`REOPENED` (fix didn't hold, back to investigating).

---

## 1. Back-left/back-right modules swapped in module array

**Status:** FIXED-UNTESTED

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`

**Problem:** The `modules[]` array was commented `// FL, FR, BL, BR` but the
constructor actually assigned it in `FL, FR, BR, BL` order, while
`getModuleTranslations()` (used to build the `SwerveDriveKinematics`) used
the order the comment claimed (`FL, FR, BL, BR`). Since `runVelocity()`
applies `kinematics.toSwerveModuleStates(...)[i]` to `modules[i]`, the
setpoint computed for the back-left wheel was being sent to the back-right
motor and vice versa. Same mismatch corrupted odometry (`getModulePositions()`
feeding the pose estimator).

**Fix:** Reordered the constructor to store modules as `FL, FR, BL, BR`,
matching the existing comment and `getModuleTranslations()`.

**Still needs:** On-robot verification that turning/curved driving behaves
correctly (back wheels no longer fighting each other).

---

## 2. Drive motor closed-loop gains were all zero

**Status:** FIXED-UNTESTED

**File:** `src/main/java/frc/robot/Constants.java`

**Problem:** Every module's `driveMotorGains` was
`Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0)`. Teleop
drive uses closed-loop `VelocityVoltage` control via these gains
(`ModuleIOTalonFX.setDriveVelocity`), so with every coefficient at zero the
requested output was 0 V regardless of commanded speed — the drive motors
could produce no torque under joystick control. This likely fully explains
why the `drive` branch was never observed to actually move.

**Fix:** Set starting values `kP=2.0, kI=0, kD=0, kS=0.1, kV=7.66` (kV
derived from the existing `kSpeedAt12Volts` = 0.5 m/s and wheel radius =
0.0508 m). These are **placeholder starting values**, not tuned — Phoenix
Tuner X reportedly doesn't work for this hardware setup, so real tuning
should go through the SysId/feedforward characterization commands already
wired into the auto chooser in `RobotContainer` (`Drive SysId
(Quasistatic/Dynamic)`, `Drive Simple FF Characterization`).

**Still needs:** Run characterization on the real robot and replace the
placeholder `kP`/`kS`/`kV` with measured values.

---

## 3. Analog encoder zero offsets unverified

**Status:** OPEN

**File:** `src/main/java/frc/robot/Constants.java`
(`AnalogEncoderConstants.FLZero/FRZero/BRZero/BLZero`)

**Problem:** Reported that these constants may have been guessed/copied
rather than measured, since Tuner X doesn't work for this configuration.
Not verifiable from code alone.

**Next step:** On the robot, manually point each wheel to a known reference
angle (e.g. straight forward) and confirm `AnalogEncoder.get()` matches the
corresponding zero constant; re-measure and update if not.

---

## 4. Drive motor stator current limit may be high for this hardware

**Status:** OPEN (not confirmed as a bug, flagged for sanity-check)

**File:** `src/main/java/frc/robot/Constants.java` (`slipCurrent` = 120.0 A
per module, used as both torque-current peak and stator current limit)

**Problem:** 120 A per drive module is a lot of current for a minibot-scale
Falcon 500 setup. Not verified against the actual PDH/breaker configuration.

**Next step:** Confirm this is appropriate for the robot's breakers/battery
before running at full commanded speed; lower if needed.

---

## 5. Fixes for #1 and #2 are not yet committed

**Status:** FIXED

**Problem:** The module-order and drive-gain fixes were sitting as
uncommitted changes on `bruce_drive`.

**Fix:** Committed on `bruce_drive` as `99d0fa6` ("Fix swerve drive bugs:
back module swap and zero drive gains"). Not pushed to the remote.
