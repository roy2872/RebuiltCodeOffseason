package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class NestedInterpolatingTreeMapTest {
  private static NestedInterpolatingTreeMap<Double, Translation3d> table() {
    NestedInterpolatingTreeMap<Double, Translation3d> table =
        new NestedInterpolatingTreeMap<>((a, b, query) -> (query - a) / (b - a), Translation3d::interpolate);
    table.put(1.0, 0.0, new Translation3d(1, 10, 100));
    table.put(2.0, 0.0, new Translation3d(2, 20, 200));
    return table;
  }

  @Test
  void exactKeyReturnsThatRow() {
    // Used to be NaN: interpolating between a key and itself divides 0 by 0
    assertEquals(new Translation3d(2, 20, 200), table().get(2.0, 0.0));
  }

  @Test
  void interpolatesBetweenKeys() {
    assertEquals(new Translation3d(1.5, 15, 150), table().get(1.5, 0.0));
  }
}
