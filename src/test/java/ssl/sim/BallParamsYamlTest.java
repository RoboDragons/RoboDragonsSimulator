package ssl.sim;

import org.junit.jupiter.api.Test;
import ssl.sim.ball.Ball;
import ssl.sim.core.BallParamsYaml;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import static org.junit.jupiter.api.Assertions.*;

class BallParamsYamlTest {
    @Test
    void everyParameterHasACommentAndRoundTrips() throws Exception {
        String y = BallParamsYaml.defaultYaml();
        String[] lines = y.split("\n");
        for (Field f : Ball.Params.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers())) continue;
            int idx = -1;
            for (int i = 0; i < lines.length; i++) if (lines[i].startsWith(f.getName() + ":")) idx = i;
            assertTrue(idx > 0, "missing " + f.getName());
            assertTrue(lines[idx - 1].startsWith("# ") && lines[idx - 1].contains("["), "no commented unit above " + f.getName());
        }
        Ball.Params p = new Ball.Params();
        p.mass = 1; p.muWall = 0.9;
        BallParamsYaml.apply(y, p);
        assertEquals(new Ball.Params().mass, p.mass);
        assertEquals(new Ball.Params().muWall, p.muWall);
        BallParamsYaml.apply("muSlide: 0.55\n", p);
        assertEquals(0.55, p.muSlide);
        assertThrows(IllegalArgumentException.class, () -> BallParamsYaml.apply("masss: 1\n", p));
    }
}
