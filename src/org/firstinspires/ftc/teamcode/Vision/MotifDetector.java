package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * MotifDetector - Vision Portal AprilTag Detection
 *
 * Detects the Motif pattern on the Obelisk to determine which zone
 * the robot should target (Left, Center, or Right).
 *
 * Uses FTC SDK VisionPortal with AprilTag processor.
 */
public class MotifDetector {

    public enum DetectedZone {
        LEFT,      // Zone 1 - Left trajectory
        CENTER,    // Zone 2 - Center trajectory
        RIGHT,     // Zone 3 - Right trajectory
        UNKNOWN    // No detection
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final AtomicReference<DetectedZone> currentZone = new AtomicReference<>(DetectedZone.UNKNOWN);

    // AprilTag IDs for DECODE Obelisk (adjust for actual season tags)
    private static final int LEFT_TAG_ID = 1;
    private static final int CENTER_TAG_ID = 2;
    private static final int RIGHT_TAG_ID = 3;

    // Minimum confidence for valid detection
    private static final double MIN_CONFIDENCE = 0.7;

    public MotifDetector() {
    }

    /**
     * Initialize the vision system with webcam
     */
    public void init(HardwareMap hardwareMap, WebcamName webcam) {
        // Build AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagOutline(true)
                .setTagFamily(org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily.TAG_36H11)
                .build();

        // Build Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcam.getName()))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUV)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    /**
     * Update detection - call this each loop iteration
     */
    public void update() {
        if (aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        DetectedZone zone = DetectedZone.UNKNOWN;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null &&
                detection.usage == AprilTagDetection.UsageValue.ALWAYS &&
                detection.rawPose != null) {

                int tagId = detection.id;

                if (tagId == LEFT_TAG_ID && detection.rawPose.z > MIN_CONFIDENCE) {
                    zone = DetectedZone.LEFT;
                } else if (tagId == CENTER_TAG_ID && detection.rawPose.z > MIN_CONFIDENCE) {
                    zone = DetectedZone.CENTER;
                } else if (tagId == RIGHT_TAG_ID && detection.rawPose.z > MIN_CONFIDENCE) {
                    zone = DetectedZone.RIGHT;
                }
            }
        }

        currentZone.set(zone);
    }

    /**
     * Get the currently detected zone
     */
    public DetectedZone getDetectedZone() {
        return currentZone.get();
    }

    /**
     * Check if a valid zone has been detected
     */
    public boolean hasValidDetection() {
        DetectedZone zone = currentZone.get();
        return zone != DetectedZone.UNKNOWN;
    }

    /**
     * Get the number of tags currently being tracked
     */
    public int getTagCount() {
        if (aprilTagProcessor == null) return 0;
        return aprilTagProcessor.getDetections().size();
    }

    /**
     * Close the vision portal when done
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}