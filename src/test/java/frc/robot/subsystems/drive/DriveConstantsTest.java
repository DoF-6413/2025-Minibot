package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class DriveConstantsTest {
  @Test
  void allEightCanIdsAreUnique() {
    List<Integer> ids =
        List.of(
            DriveConstants.kFrontLeft.driveMotorId(), DriveConstants.kFrontLeft.turnMotorId(),
            DriveConstants.kFrontRight.driveMotorId(), DriveConstants.kFrontRight.turnMotorId(),
            DriveConstants.kBackLeft.driveMotorId(), DriveConstants.kBackLeft.turnMotorId(),
            DriveConstants.kBackRight.driveMotorId(), DriveConstants.kBackRight.turnMotorId());
    assertEquals(8, Set.copyOf(ids).size(), "All 8 drive/turn CAN IDs must be unique");
  }

  @Test
  void allFourEncoderChannelsAreUnique() {
    Set<Integer> channels =
        Set.of(
            DriveConstants.kFrontLeft.encoderChannel(),
            DriveConstants.kFrontRight.encoderChannel(),
            DriveConstants.kBackLeft.encoderChannel(),
            DriveConstants.kBackRight.encoderChannel());
    assertEquals(4, channels.size(), "All 4 analog encoder channels must be unique");
  }

  @Test
  void moduleLayoutIsSymmetric() {
    assertEquals(
        DriveConstants.kFrontLeft.xPosMeters(), DriveConstants.kFrontRight.xPosMeters(), 1e-9);
    assertEquals(
        DriveConstants.kBackLeft.xPosMeters(), DriveConstants.kBackRight.xPosMeters(), 1e-9);
    assertEquals(
        -DriveConstants.kFrontLeft.xPosMeters(), DriveConstants.kBackLeft.xPosMeters(), 1e-9);
    assertEquals(
        DriveConstants.kFrontLeft.yPosMeters(), DriveConstants.kBackLeft.yPosMeters(), 1e-9);
    assertEquals(
        -DriveConstants.kFrontLeft.yPosMeters(), DriveConstants.kFrontRight.yPosMeters(), 1e-9);
  }

  @Test
  void physicalConstantsArePositive() {
    assertTrue(DriveConstants.kWheelRadiusMeters > 0);
    assertTrue(DriveConstants.kDriveGearRatio > 0);
    assertTrue(DriveConstants.kTurnGearRatio > 0);
    assertTrue(DriveConstants.kMaxLinearSpeedMetersPerSec > 0);
  }

  @Test
  void maxLinearSpeedIsInPlausibleRange() {
    // Sanity bound only — kMaxLinearSpeedMetersPerSec is a derived theoretical
    // value, not a measured one. This catches unit-conversion mistakes (e.g.
    // forgetting to divide by the gear ratio) without asserting an exact number.
    assertTrue(DriveConstants.kMaxLinearSpeedMetersPerSec > 1.0);
    assertTrue(DriveConstants.kMaxLinearSpeedMetersPerSec < 10.0);
  }
}
