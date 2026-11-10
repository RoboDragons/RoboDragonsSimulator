package ssl.sim;

import org.junit.jupiter.api.Test;
import ssl.sim.core.*;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import static org.junit.jupiter.api.Assertions.*;

class FieldParamsYamlTest {
    @Test
    void defaultIsDivisionAWithGoalMarginAndPartitions() {
        SimConfig c = new SimConfig();
        assertEquals(FieldGeometry.Division.A, c.divisionEnum());
        FieldGeometry f = c.field();
        assertEquals(0.6, f.boundaryGoalLine, 1e-9);
        assertEquals(0.3, f.boundaryTouchLine, 1e-9);
        assertTrue(f.goalSideWallsToBoundary);
        // a side wall of the goal reaches the outer wall: the visible walls contain one spanning x = hl .. hl + 0.6
        boolean found = false;
        for (var w : f.walls()) {
            double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
            for (var v : w.vertices()) {
                minX = Math.min(minX, v.getX()); maxX = Math.max(maxX, v.getX());
                minY = Math.min(minY, v.getY()); maxY = Math.max(maxY, v.getY());
            }
            if (minX < 0.0 || maxY - minY > 0.05) continue;
            if (Math.abs(minX - f.halfLength()) < 1e-9 && maxX >= f.halfLength() + 0.6 - 1e-9 && Math.abs(minY) > 0.8) found = true;
        }
        assertTrue(found, "goal side wall should extend to the outer wall");
        assertFalse(new FieldGeometry(FieldGeometry.Division.B).goalSideWallsToBoundary);
    }

    @Test
    void everyParameterHasACommentAndRoundTrips() throws Exception {
        String y = FieldParamsYaml.defaultYaml(FieldGeometry.Division.A);
        String[] lines = y.split("\n");
        for (Field f : FieldParams.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers())) continue;
            int idx = -1;
            for (int i = 0; i < lines.length; i++) if (lines[i].startsWith(f.getName() + ":")) idx = i;
            assertTrue(idx > 0, "missing " + f.getName());
            assertTrue(lines[idx - 1].startsWith("# ") && lines[idx - 1].contains("["), "no commented unit above " + f.getName());
        }
        FieldParams p = FieldParams.defaultsFor(FieldGeometry.Division.B);
        FieldParamsYaml.apply(y, p);
        FieldParams a = FieldParams.defaultsFor(FieldGeometry.Division.A);
        assertEquals(a.length, p.length);
        assertEquals(a.boundaryGoalLine, p.boundaryGoalLine);
        assertTrue(p.goalSideWallsToBoundary);
        FieldParamsYaml.apply("boundaryGoalLine: 0.45\ngoalSideWallsToBoundary: false\n", p);
        assertEquals(0.45, p.boundaryGoalLine);
        assertFalse(p.goalSideWallsToBoundary);
        assertThrows(IllegalArgumentException.class, () -> FieldParamsYaml.apply("lenght: 1\n", p));
    }
}
