package ssl.sim.viewer;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.Simulator;
import ssl.sim.core.World;
import ssl.sim.physics.ConvexPolygon;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotCommand;
import ssl.sim.robot.RobotSpecs;
import ssl.sim.vision.Camera;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL33C.*;
import static org.lwjgl.system.MemoryUtil.NULL;

/**
 * OpenGL 3.3 core 3D viewer. Deliberately light-weight (low-poly meshes, one simple shader, no shadows
 * or post effects) so that it runs on integrated graphics.
 * <pre>
 * Mouse: left drag = orbit, right/middle drag = pan, wheel = zoom
 * 1..4 camera presets (top, angled, side, follow selected robot)     V toggle vision cameras
 * Space pause   [ / ] sim speed /2, x2   B ball to centre
 * Tab next robot   T switch team   M manual control of selected robot
 *   manual: W/S forward/back, A/D strafe, Q/E rotate, R dribbler on/off, F kick, C chip
 * Device colours: chipper plate blue-grey/orange(touching ball)/red(chip fired); roller grey/green/orange = off/spinning/touching ball; kicker plate teal (charge) / orange (pending) / red (fired);
 * wheels purple (brighter = faster).
 * </pre>
 */
public class Viewer {
    /** Snapshot of everything needed to draw a robot; device geometry and state are read from the model. */
    private record RobotView(Robot.Team team, int id, double x, double y, double theta, RobotSpecs specs,
                             List<Mesh> body, double[] wheelOmega,
                             // dribbler (geometry from Dribbler, state from its motor / contact)
                             double rollerRadius, double rollerX, double rollerZ, double rollerHalfLength,
                             double rollerOmega, boolean dribblerActive, boolean dribblerContact,
                             // kicker (geometry from Kicker, state from capacitor / last kick)
                             double plateX, double plateHalfWidth, double plateZ, double plateHeight,
                             double plateThickness, double kickCharge, double kickFlash, boolean kickPending,
                             // chipper ramp plate (geometry from Chipper)
                             boolean chipperEnabled, double[] chipTip, double[] chipTop, double chipHalfWidth,
                             boolean chipperContact, double chipFlash) {}

    private final Simulator sim;
    private long window;
    private int program, uViewProj, uModel, uColor, uLight;
    private Mesh cube, cylinder, sphere;
    private final Map<Robot, List<Mesh>> bodyMeshes = new HashMap<>();
    private final Map<String, double[]> rollerAngle = new HashMap<>();

    // camera
    private double yaw = -Math.PI / 2, pitch = 0.9, dist = 9.0;
    private double targetX, targetY;
    private double lastMx, lastMy;
    private boolean dragL, dragR;
    private int preset = 0;

    // control
    private Robot.Team selTeam = Robot.Team.BLUE;
    private int selId = 0;
    private boolean manual;
    private boolean dribblerOn;
    private boolean kickRequest, chipRequest;
    private boolean showCameras = true;
    private int width = 1280, height = 720;

    private String screenshotPath;
    private int screenshotAfterFrames = 90, screenshotFrameCount;

    public Viewer(Simulator sim) { this.sim = sim; }

    /** Save a PNG after a short warm-up and exit (used for automated checks). */
    public Viewer screenshot(String path) {
        this.screenshotPath = path;
        return this;
    }

    public Viewer selectRobot(Robot.Team team, int id) {
        selTeam = team;
        selId = id;
        return this;
    }

    public Viewer cameraPreset(int p) {
        preset = p;
        return this;
    }

    private void saveScreenshot(String path) {
        java.nio.ByteBuffer buf = org.lwjgl.BufferUtils.createByteBuffer(width * height * 4);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buf);
        java.awt.image.BufferedImage img = new java.awt.image.BufferedImage(width, height, java.awt.image.BufferedImage.TYPE_INT_RGB);
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                int i = ((height - 1 - y) * width + x) * 4;
                img.setRGB(x, y, ((buf.get(i) & 0xff) << 16) | ((buf.get(i + 1) & 0xff) << 8) | (buf.get(i + 2) & 0xff));
            }
        try {
            javax.imageio.ImageIO.write(img, "png", new java.io.File(path));
            System.out.println("[viewer] screenshot saved: " + path);
        } catch (java.io.IOException e) {
            System.err.println("[viewer] cannot save screenshot: " + e);
        }
    }

    public void run() {
        init();
        double lastTime = glfwGetTime(), titleTime = 0;
        int frames = 0;
        double fps = 0;
        while (!glfwWindowShouldClose(window)) {
            double now = glfwGetTime(), dt = now - lastTime;
            lastTime = now;
            glfwPollEvents();
            applyManualControl();
            render(dt);
            if (screenshotPath != null && ++screenshotFrameCount >= screenshotAfterFrames) {
                saveScreenshot(screenshotPath);
                glfwSetWindowShouldClose(window, true);
            }
            glfwSwapBuffers(window);
            frames++;
            if (now - titleTime > 0.5) {
                fps = frames / (now - titleTime);
                frames = 0;
                titleTime = now;
                glfwSetWindowTitle(window, title(fps));
            }
        }
        cleanup();
    }

    // ------------------------------------------------------------------ init

    private void init() {
        GLFWErrorCallback.createPrint(System.err).set();
        if (!glfwInit()) throw new IllegalStateException("Unable to initialise GLFW");
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        window = glfwCreateWindow(width, height, "RoboDragons SSL Simulator", NULL, NULL);
        if (window == NULL) throw new IllegalStateException("Failed to create the window (OpenGL 3.3 required)");
        GLFWVidMode vm = glfwGetVideoMode(glfwGetPrimaryMonitor());
        if (vm != null) glfwSetWindowPos(window, (vm.width() - width) / 2, (vm.height() - height) / 2);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        GL.createCapabilities();

        glfwSetFramebufferSizeCallback(window, (w, ww, hh) -> {
            width = Math.max(1, ww);
            height = Math.max(1, hh);
            glViewport(0, 0, width, height);
        });
        glfwSetScrollCallback(window, (w, dx, dy) -> dist = Math.max(0.5, Math.min(40, dist * Math.pow(0.9, dy))));
        glfwSetMouseButtonCallback(window, (w, button, action, mods) -> {
            boolean down = action == GLFW_PRESS;
            if (button == GLFW_MOUSE_BUTTON_LEFT) dragL = down;
            else dragR = down;
            double[] x = new double[1], y = new double[1];
            glfwGetCursorPos(window, x, y);
            lastMx = x[0];
            lastMy = y[0];
        });
        glfwSetCursorPosCallback(window, (w, x, y) -> {
            double dx = x - lastMx, dy = y - lastMy;
            lastMx = x;
            lastMy = y;
            if (dragL) {
                yaw -= dx * 0.005;
                pitch = Math.max(0.05, Math.min(1.55, pitch + dy * 0.005));
                preset = 0;
            } else if (dragR) {
                double s = dist * 0.0015;
                targetX += (-Math.cos(yaw + Math.PI / 2) * dx + Math.cos(yaw) * dy) * s * -1;
                targetY += (-Math.sin(yaw + Math.PI / 2) * dx + Math.sin(yaw) * dy) * s * -1;
            }
        });
        glfwSetKeyCallback(window, (w, key, sc, action, mods) -> {
            if (action == GLFW_PRESS) onKey(key, mods);
        });
        int[] fw = new int[1], fh = new int[1];
        glfwGetFramebufferSize(window, fw, fh);
        width = fw[0];
        height = fh[0];
        glViewport(0, 0, width, height);

        program = buildProgram();
        uViewProj = glGetUniformLocation(program, "uViewProj");
        uModel = glGetUniformLocation(program, "uModel");
        uColor = glGetUniformLocation(program, "uColor");
        uLight = glGetUniformLocation(program, "uLight");
        cube = Mesh.cube();
        cylinder = Mesh.cylinder(20);
        sphere = Mesh.sphere(8, 12);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.12f, 0.14f, 0.17f, 1f);
    }

    private static int buildProgram() {
        String vs = """
                #version 330 core
                layout(location=0) in vec3 aPos;
                layout(location=1) in vec3 aNormal;
                uniform mat4 uViewProj;
                uniform mat4 uModel;
                out vec3 vN;
                void main() {
                    vec4 w = uModel * vec4(aPos, 1.0);
                    vN = mat3(transpose(inverse(uModel))) * aNormal;
                    gl_Position = uViewProj * w;
                }""";
        String fs = """
                #version 330 core
                in vec3 vN;
                uniform vec3 uColor;
                uniform vec3 uLight;
                out vec4 o;
                void main() {
                    vec3 n = normalize(vN);
                    float d = max(dot(n, normalize(uLight)), 0.0);
                    float hemi = 0.5 + 0.5 * n.z;
                    o = vec4(uColor * (0.40 * hemi + 0.70 * d), 1.0);
                }""";
        int v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
        int p = glCreateProgram();
        glAttachShader(p, v);
        glAttachShader(p, f);
        glLinkProgram(p);
        if (glGetProgrami(p, GL_LINK_STATUS) == GL_FALSE) throw new IllegalStateException(glGetProgramInfoLog(p));
        glDeleteShader(v);
        glDeleteShader(f);
        return p;
    }

    private static int compile(int type, String src) {
        int s = glCreateShader(type);
        glShaderSource(s, src);
        glCompileShader(s);
        if (glGetShaderi(s, GL_COMPILE_STATUS) == GL_FALSE) throw new IllegalStateException(glGetShaderInfoLog(s));
        return s;
    }

    private void cleanup() {
        cube.close();
        cylinder.close();
        sphere.close();
        bodyMeshes.values().forEach(l -> l.forEach(Mesh::close));
        glDeleteProgram(program);
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    // ------------------------------------------------------------------ input

    private void onKey(int key, int mods) {
        World w = sim.world();
        switch (key) {
            case GLFW_KEY_ESCAPE -> glfwSetWindowShouldClose(window, true);
            case GLFW_KEY_SPACE -> sim.setPaused(!sim.paused());
            case GLFW_KEY_LEFT_BRACKET -> sim.setSpeed(sim.speed() / 2);
            case GLFW_KEY_RIGHT_BRACKET -> sim.setSpeed(Math.max(0.0625, sim.speed()) * 2);
            case GLFW_KEY_V -> showCameras = !showCameras;
            case GLFW_KEY_1 -> preset = 1;
            case GLFW_KEY_2 -> preset = 2;
            case GLFW_KEY_3 -> preset = 3;
            case GLFW_KEY_4 -> preset = 4;
            case GLFW_KEY_M -> {
                manual = !manual;
                if (!manual) stopSelected();
            }
            case GLFW_KEY_R -> dribblerOn = !dribblerOn;
            case GLFW_KEY_F -> kickRequest = true;
            case GLFW_KEY_C -> chipRequest = true;
            case GLFW_KEY_T -> {
                stopSelected();
                selTeam = selTeam == Robot.Team.BLUE ? Robot.Team.YELLOW : Robot.Team.BLUE;
            }
            case GLFW_KEY_TAB -> {
                stopSelected();
                List<Robot> team = w.robots(selTeam);
                if (!team.isEmpty()) {
                    team.sort((a, b) -> Integer.compare(a.id, b.id));
                    int next = 0;
                    for (int i = 0; i < team.size(); i++) if (team.get(i).id > selId) { next = i; break; }
                    selId = team.get(next).id;
                }
            }
            case GLFW_KEY_B -> locked(() -> w.ball().teleport(new org.apache.commons.math3.geometry.euclidean.threed.Vector3D(0, 0, w.ball().radius()),
                    org.apache.commons.math3.geometry.euclidean.threed.Vector3D.ZERO, true));
            default -> { }
        }
    }

    private void locked(Runnable r) {
        World w = sim.world();
        w.lock().lock();
        try {
            r.run();
        } finally {
            w.lock().unlock();
        }
    }

    private Robot selected() { return sim.world().findRobot(selTeam, selId); }

    private void stopSelected() {
        Robot r = selected();
        if (r != null && manual) locked(() -> r.applyCommand(RobotCommand.stop()));
    }

    private boolean key(int k) { return glfwGetKey(window, k) == GLFW_PRESS; }

    private void applyManualControl() {
        if (!manual) return;
        Robot r = selected();
        if (r == null) return;
        double fwd = (key(GLFW_KEY_W) ? 1 : 0) - (key(GLFW_KEY_S) ? 1 : 0);
        double left = (key(GLFW_KEY_A) ? 1 : 0) - (key(GLFW_KEY_D) ? 1 : 0);
        double rot = (key(GLFW_KEY_Q) ? 1 : 0) - (key(GLFW_KEY_E) ? 1 : 0);
        double speed = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ? 3.0 : 1.2;
        double kick = 0, angle = 0;
        if (kickRequest) kick = 5.0;
        if (chipRequest) {
            kick = 4.5;
            angle = 45;
        }
        kickRequest = chipRequest = false;
        RobotCommand cmd = new RobotCommand(new RobotCommand.LocalVelocity(fwd * speed, left * speed, rot * 3.0),
                kick, angle, dribblerOn ? 9000 : 0);
        locked(() -> r.applyCommand(cmd));
    }

    // ------------------------------------------------------------------ rendering

    private String title(double fps) {
        return String.format("RoboDragons SSL Sim | Div %s | t=%.1fs x%.2f%s | sel %s %d%s%s | %.0f fps",
                sim.world().field().division, sim.world().time(), sim.speed(), sim.paused() ? " PAUSED" : "",
                selTeam, selId, manual ? " MANUAL" : "", manual && dribblerOn ? " DRIBBLER" : "", fps);
    }

    private void render(double dt) {
        World w = sim.world();
        FieldGeometry f = w.field();
        List<RobotView> robots = new ArrayList<>();
        double bx, by, bz;
        w.lock().lock();
        try {
            for (Robot r : w.robots()) {
                double[] wo = new double[r.drive().wheels().length];
                for (int i = 0; i < wo.length; i++) wo[i] = r.drive().wheels()[i].angularVelocity();
                List<Mesh> body = bodyMeshes.computeIfAbsent(r, rr -> {
                    List<Mesh> l = new ArrayList<>();
                    for (ConvexPolygon piece : rr.shape().ballPieces()) l.add(Mesh.prism(piece.vertices(), 0.02, rr.specs().height));
                    return l;
                });
                var d = r.dribbler();
                var k = r.kicker();
                robots.add(new RobotView(r.team, r.id, r.x(), r.y(), r.heading(), r.specs(), body, wo,
                        d.rollerRadius(), d.axisX(), d.axisZ(), d.halfLength(), d.rollerAngularVelocity(),
                        d.targetRpm() > 0, d.hasBallContact(),
                        k.plateX(), k.plateHalfWidth(), k.plateCenterZ(), k.plateHeight(), k.plateThickness(),
                        k.charge(), k.lastKickWasChip() ? 0 : k.kickFlash(w.time(), 0.25), k.hasPendingKick(),
                        r.chipper().enabled(), r.chipper().tip(), r.chipper().top(), r.chipper().halfWidth(),
                        r.chipper().hasBallContact(), k.lastKickWasChip() ? k.kickFlash(w.time(), 0.25) : 0));
            }
            bodyMeshes.keySet().removeIf(r -> {
                if (w.robots().contains(r)) return false;
                bodyMeshes.get(r).forEach(Mesh::close);
                return true;
            });
            bx = w.ball().x();
            by = w.ball().y();
            bz = w.ball().z();
        } finally {
            w.lock().unlock();
        }
        updateCameraPreset(robots);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program);
        double cx = targetX + dist * Math.cos(pitch) * Math.cos(yaw), cy = targetY + dist * Math.cos(pitch) * Math.sin(yaw);
        double cz = dist * Math.sin(pitch);
        float[] view = Mat4.lookAt(cx, cy, cz, targetX, targetY, 0, 0, 0, 1);
        float[] proj = Mat4.perspective(Math.toRadians(50), (double) width / height, 0.05, 200);
        glUniformMatrix4fv(uViewProj, false, Mat4.mul(proj, view));
        glUniform3f(uLight, 0.4f, -0.3f, 1.0f);

        drawField(f);
        // ball shadow + ball
        double r = w.ball().radius();
        draw(cylinder, Mat4.mul(Mat4.translate(bx, by, 0.0012), Mat4.scale(r * 1.1, r * 1.1, 0.0005)), 0.05f, 0.30f, 0.07f);
        draw(sphere, Mat4.mul(Mat4.translate(bx, by, bz), Mat4.scale(r, r, r)), 1.0f, 0.5f, 0.05f);

        for (RobotView rv : robots) drawRobot(rv, dt);
        if (showCameras) for (Camera c : sim.vision().cameras()) {
            var p = c.position();
            draw(cube, Mat4.mul(Mat4.translate(p.getX(), p.getY(), p.getZ()), Mat4.scale(0.15, 0.1, 0.1)), 0.6f, 0.6f, 0.65f);
        }
    }

    private void updateCameraPreset(List<RobotView> robots) {
        FieldGeometry f = sim.world().field();
        switch (preset) {
            case 1 -> { yaw = -Math.PI / 2; pitch = 1.5; dist = f.length * 0.95; targetX = 0; targetY = 0; }
            case 2 -> { yaw = -Math.PI / 2; pitch = 0.8; dist = f.length * 0.9; targetX = 0; targetY = 0; }
            case 3 -> { yaw = -Math.PI / 2; pitch = 0.25; dist = f.length * 0.9; targetX = 0; targetY = 0; }
            case 4 -> {
                for (RobotView rv : robots) if (rv.team() == selTeam && rv.id() == selId) {
                    targetX = rv.x();
                    targetY = rv.y();
                    yaw = rv.theta() + 0.9;
                    pitch = 0.35;
                    dist = 0.4;
                }
            }
            default -> { }
        }
        if (preset >= 1 && preset <= 3) preset = 0; // presets are one-shot; afterwards the mouse takes over
    }

    private void drawField(FieldGeometry f) {
        double hl = f.halfLength(), hw = f.halfWidth(), bx = f.boundaryGoalLine, by = f.boundaryTouchLine;
        draw(cube, Mat4.mul(Mat4.translate(0, 0, -0.023), Mat4.scale(f.length + 2 * bx + 2, f.width + 2 * by + 2, 0.02)), 0.10f, 0.12f, 0.12f);
        draw(cube, Mat4.mul(Mat4.translate(0, 0, -0.005), Mat4.scale(f.length + 2 * bx, f.width + 2 * by, 0.01)), 0.10f, 0.42f, 0.16f);
        float[] white = {0.95f, 0.95f, 0.95f};
        double t = f.lineThickness, z = 0.0008;
        line(-hl, hw, hl, hw, t, z, white);
        line(-hl, -hw, hl, -hw, t, z, white);
        line(-hl, -hw, -hl, hw, t, z, white);
        line(hl, -hw, hl, hw, t, z, white);
        line(0, -hw, 0, hw, t, z, white);
        line(-hl, 0, hl, 0, t, z, white);
        for (int s = -1; s <= 1; s += 2) {
            double xp = s * (hl - f.penaltyDepth), pw = f.penaltyWidth / 2;
            line(xp, -pw, xp, pw, t, z, white);
            line(s * hl, -pw, xp, -pw, t, z, white);
            line(s * hl, pw, xp, pw, t, z, white);
        }
        int n = 48;
        for (int i = 0; i < n; i++) {
            double a0 = 2 * Math.PI * i / n, a1 = 2 * Math.PI * (i + 1) / n, rr = f.centerCircleRadius;
            line(rr * Math.cos(a0), rr * Math.sin(a0), rr * Math.cos(a1), rr * Math.sin(a1), t, z, white);
        }
        for (ConvexPolygon wall : f.walls()) {
            double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
            for (Vector2D p : wall.vertices()) {
                minX = Math.min(minX, p.getX());
                maxX = Math.max(maxX, p.getX());
                minY = Math.min(minY, p.getY());
                maxY = Math.max(maxY, p.getY());
            }
            double cxw = (minX + maxX) / 2;
            boolean goalWall = Math.abs(cxw) > hl && Math.abs(minY) < f.goalWidth;
            boolean blueGoal = cxw < 0;
            float[] col = !goalWall || (maxX - minX > 1 && maxY - minY > 1) ? new float[]{0.30f, 0.30f, 0.34f}
                    : blueGoal ? new float[]{0.2f, 0.35f, 0.95f} : new float[]{0.95f, 0.85f, 0.15f};
            if (Math.abs(cxw) > hl && Math.abs(cxw) < hl + f.goalDepth + 0.05 + f.wallThickness && (maxY - minY) < f.goalWidth + 0.2)
                col = blueGoal ? new float[]{0.2f, 0.35f, 0.95f} : new float[]{0.95f, 0.85f, 0.15f};
            draw(cube, Mat4.mul(Mat4.translate(cxw, (minY + maxY) / 2, f.wallHeight / 2),
                    Mat4.scale(maxX - minX, maxY - minY, f.wallHeight)), col[0], col[1], col[2]);
        }
    }

    private void line(double x1, double y1, double x2, double y2, double thick, double z, float[] c) {
        double cx = (x1 + x2) / 2, cy = (y1 + y2) / 2, len = Math.hypot(x2 - x1, y2 - y1), ang = Math.atan2(y2 - y1, x2 - x1);
        draw(cube, Mat4.mul(Mat4.mul(Mat4.translate(cx, cy, z), Mat4.rotZ(ang)), Mat4.scale(len + thick * 0.5, thick, 0.0016)), c[0], c[1], c[2]);
    }

    /** SSL-Vision style marker pattern bits for ids 0..15 (4 corner dots: pink = 1, green = 0). */
    private static final int[] PATTERN = {0b1010, 0b1000, 0b0001, 0b0010, 0b1011, 0b0011, 0b1001, 0b1111,
            0b0101, 0b0100, 0b0110, 0b0111, 0b1110, 0b1100, 0b0000, 0b1101};

    private void drawRobot(RobotView rv, double dt) {
        RobotSpecs s = rv.specs();
        float[] team = rv.team() == Robot.Team.YELLOW ? new float[]{0.98f, 0.85f, 0.1f} : new float[]{0.15f, 0.35f, 0.95f};
        float[] base = Mat4.mul(Mat4.translate(rv.x(), rv.y(), 0), Mat4.rotZ(rv.theta()));
        for (Mesh m : rv.body()) {
            glUniformMatrix4fv(uModel, false, base);
            glUniform3f(uColor, 0.30f, 0.30f, 0.34f);
            m.draw();
        }
        // top plate with team colour and id dots
        draw(cylinder, Mat4.mul(base, Mat4.mul(Mat4.translate(0, 0, s.height + 0.001), Mat4.scale(0.075, 0.075, 0.003))), team[0], team[1], team[2]);
        int pat = PATTERN[rv.id() & 15];
        double[] ang = {Math.PI / 4, 3 * Math.PI / 4, -3 * Math.PI / 4, -Math.PI / 4}; // FL, BL, BR, FR
        for (int i = 0; i < 4; i++) {
            boolean pink = ((pat >> (3 - i)) & 1) == 1;
            draw(cylinder, Mat4.mul(base, Mat4.mul(Mat4.translate(0.052 * Math.cos(ang[i]), 0.052 * Math.sin(ang[i]), s.height + 0.003),
                    Mat4.scale(0.0125, 0.0125, 0.002))), pink ? 1f : 0.1f, pink ? 0.2f : 0.85f, pink ? 0.7f : 0.2f);
        }
        draw(cube, Mat4.mul(base, Mat4.mul(Mat4.translate(0.045, 0, s.height + 0.0035), Mat4.scale(0.02, 0.01, 0.002))), 1f, 1f, 1f);
        // wheels: purple, brighter the faster they spin
        for (int i = 0; i < s.wheelAngles.length; i++) {
            float k = (float) Math.min(1.0, Math.abs(rv.wheelOmega()[i]) / 150.0);
            float[] wm = Mat4.mul(base, Mat4.mul(Mat4.rotZ(s.wheelAngles[i]), Mat4.translate(s.wheelMountRadius + 0.004, 0, s.wheelRadius)));
            wm = Mat4.mul(wm, Mat4.mul(Mat4.rotY(Math.PI / 2), Mat4.scale(s.wheelRadius, s.wheelRadius, 0.02)));
            draw(cylinder, wm, 0.30f + 0.45f * k, 0.10f, 0.45f + 0.5f * k);
        }
        // kicker plate (sits at the floor of the dribbler notch): dark teal = idle/charging (brightness = charge),
        // orange = kick requested but not yet fired, red and pushed forward = just fired
        float[] kc;
        double ext = rv.kickFlash() * 0.015;
        if (rv.kickFlash() > 0) kc = new float[]{1.0f, 0.1f, 0.1f};
        else if (rv.kickPending()) kc = new float[]{1.0f, 0.55f, 0.1f};
        else {
            float c = (float) rv.kickCharge();
            kc = new float[]{0.05f, 0.20f + 0.45f * c, 0.25f + 0.45f * c};
        }
        draw(cube, Mat4.mul(base, Mat4.mul(Mat4.translate(rv.plateX() + rv.plateThickness() / 2 + ext, 0, rv.plateZ()),
                Mat4.scale(rv.plateThickness(), 2 * rv.plateHalfWidth(), rv.plateHeight()))), kc[0], kc[1], kc[2]);
        // chipper ramp plate: blue-grey = idle, orange = touching the ball (3-point hold), red = chip kick just fired,
        // hidden when disabled (2-point hold)
        if (rv.chipperEnabled()) {
            double[] a = rv.chipTip(), b = rv.chipTop();
            double dx = b[0] - a[0], dz = b[1] - a[1], len = Math.hypot(dx, dz);
            float[] cc = rv.chipFlash() > 0 ? new float[]{1.0f, 0.1f, 0.1f}
                    : rv.chipperContact() ? new float[]{1.0f, 0.55f, 0.1f} : new float[]{0.35f, 0.45f, 0.75f};
            float[] cm = Mat4.mul(base, Mat4.mul(Mat4.translate((a[0] + b[0]) / 2, 0, (a[1] + b[1]) / 2),
                    Mat4.mul(Mat4.rotY(Math.atan2(-dz, dx)), Mat4.scale(len, 2 * rv.chipHalfWidth(), 0.003))));
            draw(cube, cm, cc[0], cc[1], cc[2]);
        }
        // dribbler roller: grey = off, green = spinning, orange = spinning and touching the ball.
        // The white stripe visualises the rotation.
        String k = rv.team() + ":" + rv.id();
        double[] phi = rollerAngle.computeIfAbsent(k, kk -> new double[1]);
        phi[0] = (phi[0] + rv.rollerOmega() * Math.min(dt, 0.05) * 0.02) % (2 * Math.PI); // slowed down for visibility
        float[] rc = !rv.dribblerActive() ? new float[]{0.35f, 0.35f, 0.38f}
                : rv.dribblerContact() ? new float[]{1.0f, 0.55f, 0.1f} : new float[]{0.15f, 0.85f, 0.25f};
        float[] rollerBase = Mat4.mul(base, Mat4.translate(rv.rollerX(), 0, rv.rollerZ()));
        draw(cylinder, Mat4.mul(rollerBase, Mat4.mul(Mat4.rotX(Math.PI / 2),
                Mat4.scale(rv.rollerRadius(), rv.rollerRadius(), 2 * rv.rollerHalfLength()))), rc[0], rc[1], rc[2]);
        float[] stripe = Mat4.mul(rollerBase, Mat4.mul(Mat4.rotY(phi[0]), Mat4.translate(rv.rollerRadius(), 0, 0)));
        draw(cube, Mat4.mul(stripe, Mat4.scale(0.004, 2 * rv.rollerHalfLength() * 0.9, 0.004)), 0.95f, 0.95f, 0.95f);
    }

    private void draw(Mesh m, float[] model, float r, float g, float b) {
        glUniformMatrix4fv(uModel, false, model);
        glUniform3f(uColor, r, g, b);
        m.draw();
    }
}
