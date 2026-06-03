package frc.lib.util;

import java.util.TreeMap;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * A two-dimensional interpolating map, using a map of maps and repeated
 * interpolation.
 * 
 * https://en.wikipedia.org/wiki/Bilinear_interpolation#Repeated_linear_interpolation
 */
public class NestedInterpolatingTreeMap<K extends Comparable<K>, V> {
    private final InverseInterpolator<K> m_keyInterpolator;
    private final Interpolator<V> m_valueInterpolator;
    private final TreeMap<K, InterpolatingTreeMap<K, V>> m_map;

    public NestedInterpolatingTreeMap(
            InverseInterpolator<K> keyInterpolator,
            Interpolator<V> valueInterpolator) {
        m_keyInterpolator = keyInterpolator;
        m_valueInterpolator = valueInterpolator;
        m_map = new TreeMap<>();
    }

    public void put(K k1, K k2, V value) {
        m_map.computeIfAbsent(k1,
                (k) -> new InterpolatingTreeMap<>(
                        m_keyInterpolator, m_valueInterpolator))
                .put(k2, value);
    }

    public V get(K k1, K k2) {
        K ceilKey1 = m_map.ceilingKey(k1);
        K floorKey1 = m_map.floorKey(k1);
        if (ceilKey1 == null && floorKey1 == null) {
            // map is empty
            return null;
        }
        if (ceilKey1 == null) {
            // k1 is beyond the top
            InterpolatingTreeMap<K, V> m = m_map.get(floorKey1);
            return m.get(k2);
        }
        if (floorKey1 == null) {
            // k1 is beyond the bottom
            InterpolatingTreeMap<K, V> m = m_map.get(ceilKey1);
            return m.get(k2);
        }
        InterpolatingTreeMap<K, V> floorMap1 = m_map.get(floorKey1);
        InterpolatingTreeMap<K, V> ceilMap1 = m_map.get(ceilKey1);
        V floorV1 = floorMap1.get(k2);
        V ceilV1 = ceilMap1.get(k2);
        double interpolatedKey1 = m_keyInterpolator.inverseInterpolate(
                floorKey1, ceilKey1, k1);
        return m_valueInterpolator.interpolate(
                floorV1, ceilV1, interpolatedKey1);
    }

}