package ssl.sim;

import org.junit.jupiter.api.Test;
import ssl.sim.core.*;

import static org.junit.jupiter.api.Assertions.*;

class CameraParamsYamlTest {
    @Test
    void generatedFileReproducesTheAutomaticLayout() {
        for (var d : FieldGeometry.Division.values()) {
            FieldGeometry f = new FieldGeometry(d);
            SimConfig auto = new SimConfig();
            SimConfig fromFile = new SimConfig();
            CameraParamsYaml.apply(CameraParamsYaml.defaultYaml(f), fromFile.cameras);
            var a = ssl.sim.vision.VisionSystem.forField(f, auto.cameras).cameras();
            var b = ssl.sim.vision.VisionSystem.forField(f, fromFile.cameras).cameras();
            assertEquals(a.size(), b.size());
            for (int i = 0; i < a.size(); i++) {
                assertEquals(a.get(i).position().getX(), b.get(i).position().getX(), 1e-9);
                assertEquals(a.get(i).focalLength(), b.get(i).focalLength(), 1e-6);
            }
        }
    }

    @Test
    void explicitPositions() {
        SimConfig c = new SimConfig();
        CameraParamsYaml.apply("positions:\n  - x: -1.0\n    y: 0.5\n    height: 5.0\n  - x: 2.0\n    y: -0.5\n", c.cameras);
        var cams = ssl.sim.vision.VisionSystem.forField(c.field(), c.cameras).cameras();
        assertEquals(2, cams.size());
        assertEquals(5.0, cams.get(0).position().getZ(), 1e-9);
        assertEquals(c.cameras.height, cams.get(1).position().getZ(), 1e-9);
        assertEquals(0.5, cams.get(0).position().getY(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> CameraParamsYaml.apply("heigth: 1\n", c.cameras));
    }
}
