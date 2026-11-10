package ssl.sim.net;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import robo_dragons_cafe.network.proto.ssl.simulation.SslVisionDetection;
import robo_dragons_cafe.network.proto.ssl.vision.VisionDetection;
import robo_dragons_cafe.network.proto.ssl.vision.VisionGeometry;
import robo_dragons_cafe.network.proto.ssl.vision.VisionWrapper;
import ssl.sim.ball.Ball;
import ssl.sim.core.FieldGeometry;
import ssl.sim.vision.Camera;
import ssl.sim.vision.VisionSystem;

/** Conversions between simulator objects and the SSL protobuf messages. */
public final class ProtoConvert {
    private ProtoConvert() {}

    // ------------------------------------------------------------------ vision

    public static VisionWrapper.Packet detectionPacket(Camera.DetectionFrame f) {
        VisionDetection.Frame.Builder b = VisionDetection.Frame.newBuilder()
                .setFrameNumber(f.frameNumber()).setTCapture(f.tCapture()).setTSent(f.tSent()).setCameraId(f.cameraId());
        for (Camera.BallDetection d : f.balls()) {
            b.addBalls(VisionDetection.Ball.newBuilder().setConfidence((float) d.confidence())
                    .setX((float) d.xMm()).setY((float) d.yMm()).setZ((float) d.z())
                    .setPixelX((float) d.pixelX()).setPixelY((float) d.pixelY()));
        }
        for (Camera.RobotDetection d : f.yellow()) b.addRobotsYellow(robot(d));
        for (Camera.RobotDetection d : f.blue()) b.addRobotsBlue(robot(d));
        return VisionWrapper.Packet.newBuilder().setDetection(b).build();
    }

    private static VisionDetection.Robot.Builder robot(Camera.RobotDetection d) {
        return VisionDetection.Robot.newBuilder().setConfidence((float) d.confidence()).setRobotId(d.id())
                .setX((float) d.xMm()).setY((float) d.yMm()).setOrientation((float) d.orientation())
                .setPixelX((float) d.pixelX()).setPixelY((float) d.pixelY()).setHeight((float) d.height());
    }

    public static SslVisionDetection.SSL_DetectionFrame simDetectionFrame(Camera.DetectionFrame f) {
        SslVisionDetection.SSL_DetectionFrame.Builder b = SslVisionDetection.SSL_DetectionFrame.newBuilder()
                .setFrameNumber(f.frameNumber()).setTCapture(f.tCapture()).setTSent(f.tSent()).setCameraId(f.cameraId());
        for (Camera.BallDetection d : f.balls()) {
            b.addBalls(SslVisionDetection.SSL_DetectionBall.newBuilder().setConfidence((float) d.confidence())
                    .setX((float) d.xMm()).setY((float) d.yMm()).setZ((float) d.z())
                    .setPixelX((float) d.pixelX()).setPixelY((float) d.pixelY()));
        }
        for (Camera.RobotDetection d : f.yellow()) b.addRobotsYellow(simRobot(d));
        for (Camera.RobotDetection d : f.blue()) b.addRobotsBlue(simRobot(d));
        return b.build();
    }

    private static SslVisionDetection.SSL_DetectionRobot.Builder simRobot(Camera.RobotDetection d) {
        return SslVisionDetection.SSL_DetectionRobot.newBuilder().setConfidence((float) d.confidence()).setRobotId(d.id())
                .setX((float) d.xMm()).setY((float) d.yMm()).setOrientation((float) d.orientation())
                .setPixelX((float) d.pixelX()).setPixelY((float) d.pixelY()).setHeight((float) d.height());
    }

    public static VisionWrapper.Packet geometryPacket(FieldGeometry f, VisionSystem vs, Ball.Params ballParams) {
        VisionGeometry.FieldSize.Builder fs = VisionGeometry.FieldSize.newBuilder()
                .setFieldLength(mm(f.length)).setFieldWidth(mm(f.width)).setGoalWidth(mm(f.goalWidth))
                .setGoalDepth(mm(f.goalDepth)).setBoundaryWidth(mm(f.boundaryTouchLine))
                .setPenaltyAreaDepth(mm(f.penaltyDepth)).setPenaltyAreaWidth(mm(f.penaltyWidth));
        double hl = f.halfLength(), hw = f.halfWidth(), pd = f.penaltyDepth, pw = f.penaltyWidth / 2;
        line(fs, "TopTouchLine", -hl, hw, hl, hw, VisionGeometry.FieldShapeType.TopTouchLine, f);
        line(fs, "BottomTouchLine", -hl, -hw, hl, -hw, VisionGeometry.FieldShapeType.BottomTouchLine, f);
        line(fs, "LeftGoalLine", -hl, -hw, -hl, hw, VisionGeometry.FieldShapeType.LeftGoalLine, f);
        line(fs, "RightGoalLine", hl, -hw, hl, hw, VisionGeometry.FieldShapeType.RightGoalLine, f);
        line(fs, "HalfwayLine", 0, -hw, 0, hw, VisionGeometry.FieldShapeType.HalfwayLine, f);
        line(fs, "CenterLine", -hl, 0, hl, 0, VisionGeometry.FieldShapeType.CenterLine, f);
        line(fs, "LeftPenaltyStretch", -hl + pd, -pw, -hl + pd, pw, VisionGeometry.FieldShapeType.LeftPenaltyStretch, f);
        line(fs, "RightPenaltyStretch", hl - pd, -pw, hl - pd, pw, VisionGeometry.FieldShapeType.RightPenaltyStretch, f);
        line(fs, "LeftFieldLeftPenaltyStretch", -hl, -pw, -hl + pd, -pw, VisionGeometry.FieldShapeType.LeftFieldLeftPenaltyStretch, f);
        line(fs, "LeftFieldRightPenaltyStretch", -hl, pw, -hl + pd, pw, VisionGeometry.FieldShapeType.LeftFieldRightPenaltyStretch, f);
        line(fs, "RightFieldLeftPenaltyStretch", hl, pw, hl - pd, pw, VisionGeometry.FieldShapeType.RightFieldLeftPenaltyStretch, f);
        line(fs, "RightFieldRightPenaltyStretch", hl, -pw, hl - pd, -pw, VisionGeometry.FieldShapeType.RightFieldRightPenaltyStretch, f);
        fs.addFieldArcs(VisionGeometry.FieldCircularArc.newBuilder().setName("CenterCircle")
                .setCenter(VisionGeometry.Vector2f.newBuilder().setX(0).setY(0)).setRadius((float) (f.centerCircleRadius * 1000))
                .setA1(0).setA2((float) (2 * Math.PI)).setThickness((float) (f.lineThickness * 1000))
                .setType(VisionGeometry.FieldShapeType.CenterCircle));

        VisionGeometry.Geometry.Builder g = VisionGeometry.Geometry.newBuilder().setField(fs);
        for (Camera c : vs.cameras()) {
            Vector3D t = c.translationMm();
            Vector3D pos = c.position();
            g.addCalib(VisionGeometry.CameraCalibration.newBuilder().setCameraId(c.id)
                    .setFocalLength((float) c.focalLength()).setPrincipalPointX((float) c.principalX())
                    .setPrincipalPointY((float) c.principalY()).setDistortion((float) c.distortion())
                    .setQ0((float) c.worldToCamera().getQ1()).setQ1((float) c.worldToCamera().getQ2())
                    .setQ2((float) c.worldToCamera().getQ3()).setQ3((float) c.worldToCamera().getQ0())
                    .setTx((float) t.getX()).setTy((float) t.getY()).setTz((float) t.getZ())
                    .setDerivedCameraWorldTx((float) (pos.getX() * 1000)).setDerivedCameraWorldTy((float) (pos.getY() * 1000))
                    .setDerivedCameraWorldTz((float) (pos.getZ() * 1000))
                    .setPixelImageWidth(c.width()).setPixelImageHeight(c.height()));
        }
        double g9 = Ball.G;
        double k = ballParams.inertiaFactor;
        g.setModels(VisionGeometry.GeometryModels.newBuilder()
                .setStraightTwoPhase(VisionGeometry.BallModelStraightTwoPhase.newBuilder()
                        .setAccSlide(-ballParams.muSlide * g9).setAccRoll(-ballParams.aRoll).setKSwitch(1.0 / (1.0 + k)))
                .setChipFixedLoss(VisionGeometry.BallModelChipFixedLoss.newBuilder()
                        .setDampingXyFirstHop(0.8).setDampingXyOtherHops(0.85).setDampingZ(ballParams.restitutionGround)));
        return VisionWrapper.Packet.newBuilder().setGeometry(g).build();
    }

    private static int mm(double m) { return (int) Math.round(m * 1000); }

    private static void line(VisionGeometry.FieldSize.Builder fs, String name, double x1, double y1, double x2, double y2,
                             VisionGeometry.FieldShapeType type, FieldGeometry f) {
        fs.addFieldLines(VisionGeometry.FieldLineSegment.newBuilder().setName(name)
                .setP1(VisionGeometry.Vector2f.newBuilder().setX((float) (x1 * 1000)).setY((float) (y1 * 1000)))
                .setP2(VisionGeometry.Vector2f.newBuilder().setX((float) (x2 * 1000)).setY((float) (y2 * 1000)))
                .setThickness((float) (f.lineThickness * 1000)).setType(type));
    }
}
