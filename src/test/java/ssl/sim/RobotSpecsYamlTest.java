package ssl.sim;

import org.junit.jupiter.api.Test;
import ssl.sim.core.RobotSpecsYaml;
import ssl.sim.robot.RobotSpecs;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import static org.junit.jupiter.api.Assertions.*;

class RobotSpecsYamlTest {
    @Test
    void everyParameterHasACommentAndRoundTrips() throws Exception {
        String y = RobotSpecsYaml.defaultYaml();
        String[] lines = y.split("\n");
        for (Field f : RobotSpecs.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) || f.getName().equals("rotationError")) continue;
            int idx = -1;
            for (int i = 0; i < lines.length; i++) if (lines[i].startsWith(f.getName() + ":")) idx = i;
            assertTrue(idx > 0, "missing " + f.getName());
            assertTrue(lines[idx - 1].startsWith("# ") && lines[idx - 1].contains("["), "no commented unit above " + f.getName());
        }
        RobotSpecs s = new RobotSpecs();
        s.mass = 99; s.rollerSpringK = 1; // changed values must be overwritten by the file
        s.rotationError = 0.5;
        RobotSpecsYaml.apply(y, s);
        RobotSpecs d = new RobotSpecs();
        assertEquals(d.mass, s.mass);
        assertEquals(d.rollerSpringK, s.rollerSpringK);
        assertEquals(d.dribblerMaterial, s.dribblerMaterial);
        assertArrayEquals(d.wheelAngles, s.wheelAngles, 1e-12);
        assertEquals(d.erVelocityTimeConstant, s.erVelocityTimeConstant, 1e-15);
        assertTrue(y.contains("velAngularMax: 10.0"), "angular rates stay in rad/s");
        assertTrue(y.contains("[-60.0, -135.0, 135.0, 60.0]"), "wheel angles in degrees");
        assertEquals(0.5, s.rotationError); // not part of the file
    }

    @Test
    void partialEntriesAndUnknownKeys() {
        RobotSpecs s = new RobotSpecs();
        RobotSpecsYaml.apply("mass: 3\ndribblerMaterial:\n  mu: 1.5\n", s);
        assertEquals(3.0, s.mass);
        assertEquals(1.5, s.dribblerMaterial.mu());
        assertEquals(new RobotSpecs().dribblerMaterial.stiffness(), s.dribblerMaterial.stiffness());
        assertThrows(IllegalArgumentException.class, () -> RobotSpecsYaml.apply("masss: 3\n", new RobotSpecs()));
    }
}
