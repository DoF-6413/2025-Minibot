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
