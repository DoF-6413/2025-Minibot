# AdvantageKit Swerve Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a working AdvantageKit swerve `Drive` subsystem for this robot's real hardware (8 Falcon 500s, 4 analog absolute encoders, NavX gyro, no CANcoders), replacing the stock AdvantageKit CTRE-swerve template's CANcoder-based module IO with an analog-encoder-seeded approach.

**Architecture:** Adapt the stock Littleton Robotics AdvantageKit CTRE-swerve template file structure (`Drive`, `Module`, `ModuleIO*`, `GyroIO*`, `PhoenixOdometryThread`) but replace CTRE's generated `TunerConstants`/`SwerveModuleConstants<>` machinery with a hand-written `DriveConstants.java`. `ModuleIOTalonFX` drops `CANcoder` entirely: the turn Falcon closes the loop on its own internal rotor sensor and is seeded at startup from a WPILib `AnalogEncoder`, exactly matching the technique already proven in this repo's `TankDrive` branch.

**Tech Stack:** Java 17, WPILib 2026, AdvantageKit (akit) 26.0.0, CTRE Phoenix 6 (26.1.0), Studica (NavX) 2026.0.0, GradleRIO, JUnit 5.

**Spec:** `docs/superpowers/specs/2026-09-08-advantagekit-swerve-drive-design.md`

## Global Constraints

- No PathPlanner / autonomous — core swerve teleop drive + odometry only (per spec's "Out of scope").
- No CANcoder anywhere — turn absolute position comes only from WPILib `AnalogEncoder` (per Research.md / spec).
- CAN bus is the roboRIO bus only — no CANivore (per spec, matches `TankDrive`).
- Reuse verbatim from `TankDrive` (do not re-derive or guess): turn CAN IDs 1/3/5/7 and drive CAN IDs 2/4/6/8 (FL=1/2, FR=3/4, BR=5/6, BL=7/8), analog encoder ports 0-3 with their exact zero offsets, drive gear ratio `6.122448979591837`, turn gear ratio `12.8`, drive gains (kP=0.1, kV=0.124), turn gains (kP=60, kV=0.124), wheel diameter 4in (2in radius), turn motor inversion Clockwise_Positive.
- Placeholder values (track width/wheelbase 11.5in square, theoretical max speed, per-corner drive inversion convention) must be marked with `// TODO: verify on robot` comments — never presented as measured fact.
- Every file's Javadoc/comment style, package layout, and control patterns should match the stock AdvantageKit template's conventions (this repo's own `dev`/`bruce_drive`/`lohitSwerve` branches show the reference pattern) so a reviewer familiar with AdvantageKit recognizes the code immediately.
- This is a Windows dev machine using Git Bash (MSYS). Any `git show <ref>:<path>` command in this plan MUST be prefixed with `MSYS_NO_PATHCONV=1` or the colon in the ref gets mangled by MSYS path conversion.

---

### Task 1: Project skeleton bootstrap

**Files:**
- Create (copied verbatim from `origin/TankDrive` via `git show`): `build.gradle`, `settings.gradle`, `gradlew`, `gradlew.bat`, `gradle/wrapper/gradle-wrapper.jar`, `gradle/wrapper/gradle-wrapper.properties`, `.wpilib/wpilib_preferences.json`, `.vscode/extensions.json`, `.vscode/settings.json`, `.vscode/launch.json`, `AdvantageKit-License.md`, `WPILib-License.md`, `vendordeps/AdvantageKit.json`, `vendordeps/Phoenix6.json`, `vendordeps/Studica.json`, `vendordeps/WPILibNewCommands.json`, `src/main/java/frc/robot/Constants.java`, `src/main/java/frc/robot/Main.java`, `src/main/java/frc/robot/Robot.java`, `src/main/java/frc/robot/util/PhoenixUtil.java`
- Create (new, temporary stub — replaced in Task 10): `src/main/java/frc/robot/RobotContainer.java`

**Interfaces:**
- Produces: a buildable, empty-drivetrain robot project. `RobotContainer` (stub) exposes `public RobotContainer()` and `public Command getAutonomousCommand()` returning `null` — later tasks add real subsystems without changing this signature until Task 10 replaces the file outright.

- [x] **Step 1: Copy the project skeleton files from `origin/TankDrive`** — DONE (commit d1f36d1)

- [x] **Step 2: Create the temporary stub `RobotContainer.java`** — DONE (commit d1f36d1)

- [x] **Step 3: Build** — DONE

- [x] **Step 4: Commit** — DONE (commits d1f36d1, 272ece0 fix for gradlew executable bit)

---

### Task 2: `DriveConstants.java`

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
- Test: `src/test/java/frc/robot/subsystems/drive/DriveConstantsTest.java`

**Interfaces:**
- Produces: `DriveConstants` (final class, package `frc.robot.subsystems.drive`) with:
  - `public static final CANBus kCANBus`
  - `public static final double kWheelRadiusMeters, kDriveGearRatio, kTurnGearRatio, kMaxLinearSpeedMetersPerSec, kDriveCurrentLimitAmps`
  - `public static final double kDriveP, kDriveI, kDriveD, kDriveS, kDriveV, kTurnP, kTurnI, kTurnD, kTurnS, kTurnV`
  - `public static final boolean kTurnMotorInverted`
  - nested `public record ModuleConstants(int driveMotorId, int turnMotorId, boolean driveMotorInverted, int encoderChannel, double encoderOffsetRadians, double xPosMeters, double yPosMeters)`
  - `public static final ModuleConstants kFrontLeft, kFrontRight, kBackLeft, kBackRight`

- [x] **Steps 1-5** — DONE (commit 037e3be), review clean.

(Full original source for this task's files lives in git commit 037e3be — see `git show 037e3be` if you need to re-derive it; this recreated plan file omits the large inline code blocks that were already implemented and reviewed, to keep this recovery pass short. Remaining, not-yet-implemented tasks below are reproduced in full.)

---

### Task 3: IO interfaces — `ModuleIO.java` + `GyroIO.java`

- [x] **Steps 1-3** — DONE (commit 785dc3c), copied verbatim from `dev`, review clean.

---

### Task 4: `Module.java`

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/Module.java`

**Interfaces:**
- Consumes: `ModuleIO` / `ModuleIOInputsAutoLogged` (Task 3), `DriveConstants.kWheelRadiusMeters` (Task 2).
- Produces: `Module` with constructor `Module(ModuleIO io, int index)` and public methods `periodic()`, `runSetpoint(SwerveModuleState state)`, `runCharacterization(double output)`, `stop()`, `getAngle(): Rotation2d`, `getPositionMeters(): double`, `getVelocityMetersPerSec(): double`, `getPosition(): SwerveModulePosition`, `getState(): SwerveModuleState`, `getOdometryPositions(): SwerveModulePosition[]`, `getOdometryTimestamps(): double[]`, `getWheelRadiusCharacterizationPosition(): double`, `getFFCharacterizationVelocity(): double` — all consumed by `Drive` in Task 5.
- Note: unlike the stock template, `Module` does **not** take a `SwerveModuleConstants<>`/per-module constants object — wheel radius is shared and read directly from `DriveConstants`, since this robot has one wheel size for all four modules.

- [ ] **Step 1: Write `Module.java`**

```java
package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Wraps a single swerve module's IO layer with kinematics-aware helpers used by {@link Drive}. */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.kWheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.kWheelRadiusMeters);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.kWheelRadiusMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
```

- [ ] **Step 2: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Module.java
git commit -m "Add Module"
```

---

### Task 5: `PhoenixOdometryThread.java` + `Drive.java`

These two are done together: `Drive` declares the static `odometryLock` and `ODOMETRY_FREQUENCY` that `PhoenixOdometryThread` (and, in Task 6/7, `GyroIONavX`/`ModuleIOTalonFX`) depend on, while `Drive`'s constructor starts the thread — neither compiles meaningfully alone.

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java`
- Create: `src/main/java/frc/robot/subsystems/drive/Drive.java`

**Interfaces:**
- Consumes: `Module` (Task 4), `ModuleIO`/`GyroIO` (Task 3), `DriveConstants` (Task 2).
- Produces:
  - `PhoenixOdometryThread` — singleton via `getInstance()`, with `start()`, `registerSignal(StatusSignal<Angle>)`, `registerSignal(DoubleSupplier)`, `makeTimestampQueue()` — consumed by `ModuleIOTalonFX` (Task 7) and `GyroIONavX` (Task 6).
  - `Drive` (extends `SubsystemBase`) — constructor `Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO)`; static `Drive.ODOMETRY_FREQUENCY`, `Drive.odometryLock` (package-private, used by `PhoenixOdometryThread`), `Drive.DRIVE_BASE_RADIUS` (public); public methods `runVelocity(ChassisSpeeds)`, `stop()`, `stopWithX()`, `sysIdQuasistatic(Direction)`, `sysIdDynamic(Direction)`, `getPose(): Pose2d`, `getRotation(): Rotation2d`, `setPose(Pose2d)`, `addVisionMeasurement(...)`, `getMaxLinearSpeedMetersPerSec(): double`, `getMaxAngularSpeedRadPerSec(): double`, `getWheelRadiusCharacterizationPositions(): double[]`, `getFFCharacterizationVelocity(): double`, static `getModuleTranslations(): Translation2d[]` — all consumed by `RobotContainer` (Task 10) and `DriveCommands` (Task 9).

- [ ] **Step 1: Write `PhoenixOdometryThread.java`**

```java
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of
 * queues. This robot has no CANivore, so this always runs in the non-CAN-FD polling mode.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static final boolean isCANFD = DriveConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      Drive.odometryLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
```

- [ ] **Step 2: Write `Drive.java`**

```java
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve drive subsystem: kinematics, odometry, and per-module setpoint dispatch. */
public class Drive extends SubsystemBase {
  static final double ODOMETRY_FREQUENCY = DriveConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  DriveConstants.kFrontLeft.xPosMeters(), DriveConstants.kFrontLeft.yPosMeters()),
              Math.hypot(
                  DriveConstants.kFrontRight.xPosMeters(),
                  DriveConstants.kFrontRight.yPosMeters())),
          Math.max(
              Math.hypot(
                  DriveConstants.kBackLeft.xPosMeters(), DriveConstants.kBackLeft.yPosMeters()),
              Math.hypot(
                  DriveConstants.kBackRight.xPosMeters(), DriveConstants.kBackRight.yPosMeters())));

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    PhoenixOdometryThread.getInstance().start();

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.kMaxLinearSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.kMaxLinearSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.kFrontLeft.xPosMeters(), DriveConstants.kFrontLeft.yPosMeters()),
      new Translation2d(
          DriveConstants.kFrontRight.xPosMeters(), DriveConstants.kFrontRight.yPosMeters()),
      new Translation2d(DriveConstants.kBackLeft.xPosMeters(), DriveConstants.kBackLeft.yPosMeters()),
      new Translation2d(
          DriveConstants.kBackRight.xPosMeters(), DriveConstants.kBackRight.yPosMeters())
    };
  }
}
```

- [ ] **Step 3: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "Add Drive subsystem and PhoenixOdometryThread"
```

---

### Task 6: `GyroIONavX.java`

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java`

**Interfaces:**
- Consumes: `GyroIO`/`GyroIOInputsAutoLogged` (Task 3), `Drive.ODOMETRY_FREQUENCY` and `PhoenixOdometryThread` (Task 5).
- Produces: `GyroIONavX implements GyroIO` — consumed by `RobotContainer` (Task 10) for the `REAL` mode gyro. No CANcoder/CAN dependency to begin with, so this is unchanged from the stock template.

- [ ] **Step 1: Copy from `dev`**

```bash
MSYS_NO_PATHCONV=1 git show dev:src/main/java/frc/robot/subsystems/drive/GyroIONavX.java > src/main/java/frc/robot/subsystems/drive/GyroIONavX.java
```

- [ ] **Step 2: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/GyroIONavX.java
git commit -m "Add GyroIONavX"
```

---

### Task 7: `ModuleIOTalonFX.java` (the Research.md deliverable)

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`

**Interfaces:**
- Consumes: `ModuleIO`/`ModuleIOInputsAutoLogged` (Task 3), `DriveConstants`/`DriveConstants.ModuleConstants` (Task 2), `PhoenixOdometryThread` + `Drive.ODOMETRY_FREQUENCY` (Task 5), `PhoenixUtil.tryUntilOk` (Task 1).
- Produces: `ModuleIOTalonFX implements ModuleIO` with constructor `ModuleIOTalonFX(DriveConstants.ModuleConstants constants)` — consumed by `RobotContainer` (Task 10) for the `REAL` mode modules.
- This is the file Research.md specifically calls for: no `CANcoder`; a WPILib `AnalogEncoder` provides the absolute position, and the Falcon's own rotor sensor is seeded from it at construction.

- [ ] **Step 1: Write `ModuleIOTalonFX.java`**

```java
package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import java.util.Queue;

/**
 * Module IO implementation for a TalonFX drive motor, TalonFX turn motor, and a WPILib {@link
 * AnalogEncoder} for absolute turn position — no CANcoder. The turn Falcon closes its position
 * loop on its own internal rotor sensor ({@code SensorToMechanismRatio} = turn gear ratio); at
 * construction its rotor position is seeded from the analog encoder's absolute reading, exactly
 * as this repo's TankDrive branch already does successfully.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final AnalogEncoder turnEncoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOTalonFX(ModuleConstants constants) {
    driveTalon = new TalonFX(constants.driveMotorId(), DriveConstants.kCANBus);
    turnTalon = new TalonFX(constants.turnMotorId(), DriveConstants.kCANBus);
    // AnalogEncoder(channel, fullRange, expectedZero): full range is 2*PI
    // radians, offset is the raw voltage-derived reading at the mechanical
    // zero position (measured on TankDrive).
    turnEncoder =
        new AnalogEncoder(
            constants.encoderChannel(), 2.0 * Math.PI, constants.encoderOffsetRadians());

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        constants.driveMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveGearRatio;
    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kDriveP)
            .withKI(DriveConstants.kDriveI)
            .withKD(DriveConstants.kDriveD)
            .withKS(DriveConstants.kDriveS)
            .withKV(DriveConstants.kDriveV);
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kDriveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted =
        DriveConstants.kTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kTurnP)
            .withKI(DriveConstants.kTurnI)
            .withKD(DriveConstants.kTurnD)
            .withKS(DriveConstants.kTurnS)
            .withKV(DriveConstants.kTurnV);
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
    // Seed the Falcon's internal rotor position from the analog encoder's
    // absolute reading (Research.md step 3).
    tryUntilOk(
        5, () -> turnTalon.setPosition(Units.radiansToRotations(turnEncoder.get()), 0.25));

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    // AnalogEncoder exposes no connection-health API, unlike a CANcoder status
    // signal — there's nothing more specific to debounce here.
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnEncoder.get());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double v) -> Units.rotationsToRadians(v))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double v) -> Rotation2d.fromRotations(v))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveTalon.setControl(
        velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionRequest.withPosition(rotation.getRotations()));
  }
}
```

- [ ] **Step 2: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

There is no meaningful pure-logic unit to isolate here beyond a single WPILib `Units.radiansToRotations` call (that would test WPILib, not our code) — correctness of this file rests on compilation, code review against the AdvantageKit/Research.md pattern (Task 11), and on-robot verification (per the spec's testing plan), not a unit test.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java
git commit -m "Add ModuleIOTalonFX with analog-encoder seeding (Research.md adaptation)"
```

---

### Task 8: `ModuleIOSim.java`

**Files:**
- Create: `src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java`

**Interfaces:**
- Consumes: `ModuleIO`/`ModuleIOInputsAutoLogged` (Task 3), `DriveConstants.kDriveGearRatio`/`kTurnGearRatio` (Task 2).
- Produces: `ModuleIOSim implements ModuleIO` with a no-arg constructor `ModuleIOSim()` — consumed by `RobotContainer` (Task 10) for the `SIM` mode modules.

- [ ] **Step 1: Write `ModuleIOSim.java`**

```java
package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO, modeled on a Falcon 500 drive + turn pair. */
public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV = 0.1;
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final double DRIVE_MOI = 0.025;
  private static final double TURN_MOI = 0.004;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getFalcon500(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getFalcon500(1);

  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, DRIVE_MOI, DriveConstants.kDriveGearRatio),
          DRIVE_GEARBOX);
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(TURN_GEARBOX, TURN_MOI, DriveConstants.kTurnGearRatio),
          TURN_GEARBOX);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
```

- [ ] **Step 2: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java
git commit -m "Add ModuleIOSim"
```

---

### Task 9: `DriveCommands.java`

**Files:**
- Create: `src/main/java/frc/robot/commands/DriveCommands.java`
- Test: `src/test/java/frc/robot/commands/DriveCommandsTest.java`

**Interfaces:**
- Consumes: `Drive` (Task 5) — `getMaxLinearSpeedMetersPerSec()`, `getMaxAngularSpeedRadPerSec()`, `runVelocity(ChassisSpeeds)`, `getRotation()`, `runCharacterization(double)`, `getFFCharacterizationVelocity()`, `getWheelRadiusCharacterizationPositions()`, `Drive.DRIVE_BASE_RADIUS`.
- Produces: `DriveCommands` (final utility class) with static methods `joystickDrive(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier): Command`, `joystickDriveAtAngle(Drive, DoubleSupplier, DoubleSupplier, Supplier<Rotation2d>): Command`, `feedforwardCharacterization(Drive): Command`, `wheelRadiusCharacterization(Drive): Command`, and package-private `static Translation2d getLinearVelocityFromJoysticks(double x, double y)` — consumed by `RobotContainer` (Task 10) and by this task's own test.

- [ ] **Step 1: Copy from `dev`, then loosen one method's visibility for testability**

```bash
mkdir -p src/main/java/frc/robot/commands
MSYS_NO_PATHCONV=1 git show dev:src/main/java/frc/robot/commands/DriveCommands.java > src/main/java/frc/robot/commands/DriveCommands.java
```

Edit the file: change

```java
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
```

to

```java
  static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
```

(package-private instead of `private`, so the test below can call it directly — everything else in the file is unchanged from `dev`, which has no PathPlanner or `TunerConstants` dependency.)

- [ ] **Step 2: Write the test for the now-visible helper**

```java
package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @Test
  void zeroInputProducesZeroVelocity() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(0.0, 0.0);
    assertEquals(0.0, result.getNorm(), 1e-9);
  }

  @Test
  void belowDeadbandProducesZeroVelocity() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(0.005, 0.0);
    assertEquals(0.0, result.getNorm(), 1e-9);
  }

  @Test
  void fullForwardInputProducesUnitMagnitudeInPositiveX() {
    Translation2d result = DriveCommands.getLinearVelocityFromJoysticks(1.0, 0.0);
    assertEquals(1.0, result.getNorm(), 1e-6);
    assertTrue(result.getX() > 0.99);
    assertEquals(0.0, result.getY(), 1e-6);
  }

  @Test
  void magnitudeIsSquaredForFinerControlAtLowInput() {
    Translation2d half = DriveCommands.getLinearVelocityFromJoysticks(0.5, 0.0);
    assertTrue(half.getNorm() < 0.4, "Expected squared response to reduce low-end magnitude");
  }
}
```

- [ ] **Step 3: Run tests**

Run: `./gradlew test --tests "frc.robot.commands.DriveCommandsTest"`
Expected: PASS (4 tests).

- [ ] **Step 4: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/commands/DriveCommands.java src/test/java/frc/robot/commands/DriveCommandsTest.java
git commit -m "Add DriveCommands"
```

---

### Task 10: `RobotContainer.java` (final wiring)

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java` (replaces the Task 1 stub entirely)

**Interfaces:**
- Consumes: `Drive`, `GyroIO`, `GyroIONavX`, `ModuleIO`, `ModuleIOSim`, `ModuleIOTalonFX`, `DriveConstants` (Tasks 2-8), `DriveCommands` (Task 9), `Constants.currentMode`/`Mode` (Task 1).
- Produces: `RobotContainer` with `public RobotContainer()` and `public Command getAutonomousCommand()` — same signature Task 1's stub used, so `Robot.java` needs no changes.

- [ ] **Step 1: Replace `RobotContainer.java`**

```java
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares subsystems, button bindings, and the autonomous/characterization chooser. */
public class RobotContainer {
  private final Drive drive;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot: TalonFX drive + turn, AnalogEncoder absolute position
        // (no CANcoder), NavX gyro.
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(DriveConstants.kFrontLeft),
                new ModuleIOTalonFX(DriveConstants.kFrontRight),
                new ModuleIOTalonFX(DriveConstants.kBackLeft),
                new ModuleIOTalonFX(DriveConstants.kBackRight));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: field-relative joystick drive.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Hold A: snap to 0 degrees while still translating with the left stick.
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Press X: lock wheels in an X pattern to resist being pushed.
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Press B: reset the gyro heading to 0 (keeps current field position).
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
```

- [ ] **Step 2: Build**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Run the full test suite**

Run: `./gradlew test`
Expected: all tests PASS (`DriveConstantsTest` from Task 2, `DriveCommandsTest` from Task 9).

- [ ] **Step 4: Desktop simulation smoke check**

Run: `./gradlew simulateJava` (headless is fine — `wpi.sim.addGui().defaultEnabled = false` in `build.gradle`), let it start, then stop it (Ctrl+C). Confirm it starts without throwing (constructs `Drive` with all 4 `ModuleIOSim` + no-op `GyroIO` successfully) and shuts down cleanly. This does not fully verify robot behavior; it verifies that the SIM code path (kinematics, pose estimator, default command wiring) actually initializes.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "Wire real/sim/replay swerve drive in RobotContainer"
```

---

### Task 11: AdvantageKit-consistency code review

Not a code-writing task — a full-diff review pass against the spec and the stock AdvantageKit template, run after Task 10 is committed.

- [ ] **Step 1: Diff review**

Review the full `claude_swerve` branch diff against `main` (`git diff main...claude_swerve`) for:
- Every placeholder value (`kTrackWidthMeters`, `kWheelBaseMeters`, `kMaxLinearSpeedMetersPerSec`, `kLeftDriveInverted`/`kRightDriveInverted`) still carries its `// TODO: verify on robot` comment and is not silently treated as measured fact anywhere else in the code or in comments.
- No `CANcoder` import or usage anywhere in `src/main/java/frc/robot/subsystems/drive/`.
- No PathPlanner import/usage anywhere (`com.pathplanner.lib.*` should not appear).
- `ModuleIOTalonFX`'s turn-motor seeding logic matches Research.md's three steps (IO field for absolute position; `AnalogEncoder` replacing `CANcoder`; `setPosition` seed converted via `Units.radiansToRotations`).
- Naming, package layout, and control-flow patterns (alerts, debouncers, `@AutoLog` inputs, `Logger.recordOutput`/`processInputs` usage, odometry-thread queue registration) match the stock AdvantageKit template used elsewhere in this repo (`dev`, `bruce_drive`, `lohitSwerve` branches) closely enough that a reviewer familiar with AdvantageKit recognizes the code immediately.
- `RobotContainer`'s real/sim/replay switch, default command, and button bindings match the stock template's shape minus PathPlanner.

- [ ] **Step 2: Fix any findings, re-run `./gradlew build && ./gradlew test`, and commit fixes**

If the review finds issues, fix them, rerun the full build/test suite, and commit with a message describing what the review caught (e.g. `git commit -m "Address AdvantageKit-consistency review findings"`). If nothing is found, no commit is needed for this task.
