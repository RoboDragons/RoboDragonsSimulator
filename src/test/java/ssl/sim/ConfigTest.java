package ssl.sim;

import org.junit.jupiter.api.Test;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.SimConfig;
import ssl.sim.core.Simulator;

import static org.junit.jupiter.api.Assertions.*;

class ConfigTest {
    @Test
    void defaultCameraCountsAndHeight() {
        SimConfig a = new SimConfig();
        a.division = "A";
        Simulator sa = new Simulator(a);
        assertEquals(2, sa.vision().cameras().size());
        assertEquals(6.0, sa.vision().cameras().get(0).position().getZ(), 1e-9);
        SimConfig b = new SimConfig();
        b.division = "B";
        Simulator sb = new Simulator(b);
        assertEquals(1, sb.vision().cameras().size());
        assertEquals(6.0, sb.vision().cameras().get(0).position().getZ(), 1e-9);
    }

    @Test
    void gsonPartialConfigOverridesOnlyGivenFields() {
        SimConfig c = SimConfig.fromJson("{\"division\":\"A\",\"cameras\":{\"count\":3,\"height\":5.5},\"robot\":{\"mass\":2.0},\"network\":{\"visionPort\":10006}}");
        assertEquals(FieldGeometry.Division.A, c.divisionEnum());
        assertEquals(3, c.cameras.effectiveCount(c.divisionEnum()));
        assertEquals(5.5, c.cameras.height, 1e-9);
        assertEquals(2.0, c.robot.mass, 1e-9);
        assertEquals(0.09, c.robot.radius, 1e-9);       // untouched default
        assertEquals(10006, c.network.visionPort);
        assertEquals(10301, c.network.bluePort);
        Simulator s = new Simulator(c);
        assertEquals(3, s.vision().cameras().size());
        assertEquals(5.5, s.vision().cameras().get(2).position().getZ(), 1e-9);
    }

    @Test
    void roundTrip() {
        SimConfig c = new SimConfig();
        SimConfig d = SimConfig.fromJson(c.toJson());
        assertEquals(c.toJson(), d.toJson());
    }

    @Test
    void camerasSeeWholeField() {
        for (var div : FieldGeometry.Division.values()) {
            SimConfig c = new SimConfig();
            c.division = div.name();
            Simulator s = new Simulator(c);
            var f = s.world().field();
            for (double x : new double[]{-f.halfLength(), 0, f.halfLength()})
                for (double y : new double[]{-f.halfWidth(), 0, f.halfWidth()}) {
                    boolean seen = false;
                    for (var cam : s.vision().cameras())
                        seen |= cam.project(new org.apache.commons.math3.geometry.euclidean.threed.Vector3D(x, y, 0.15)) != null;
                    assertTrue(seen, div + " point " + x + "," + y);
                }
        }
    }

    @Test
    void realismPresetsAndOverrides() {
        SimConfig d = new SimConfig();                                   // default: ER-Force "Realistic"
        assertEquals("Realistic", d.realism);
        assertEquals(0.035, d.cameras.latencySeconds, 1e-9);
        assertEquals(1.3, d.cameras.positionNoiseMm, 1e-9);
        assertEquals(0.5, d.robot.rotationError, 1e-9);
        assertEquals("ER_FORCE", d.robot.driveModel);
        SimConfig n = SimConfig.fromJson("{\"realism\":\"None\"}");
        assertEquals(0.0, n.cameras.positionNoiseMm, 1e-9);
        assertEquals(0.035, n.cameras.latencySeconds, 1e-9);
        SimConfig o = SimConfig.fromJson("{\"realism\":\"Friendly\",\"cameras\":{\"latencySeconds\":0.08}}");
        assertEquals(0.08, o.cameras.latencySeconds, 1e-9);              // explicit field wins
        assertEquals(0.3, o.cameras.positionNoiseMm, 1e-9);              // other fields keep the preset
        assertEquals(0.01, o.network.robotCommandLoss, 1e-9);
        SimConfig m = SimConfig.fromJson("{\"realism\":null,\"cameras\":{\"count\":3}}");
        assertEquals(1.0, m.cameras.positionNoiseMm, 1e-9);              // no preset: field defaults
    }
}
