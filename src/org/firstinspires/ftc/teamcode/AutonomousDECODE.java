package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Vision.MotifDetector;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * AutonomousDECODE - 30-Second Autonomous Routine
 *
 * State Machine Flow:
 * INIT → DETECT_ZONE → SELECT_TRAJECTORY → PRELOAD_SCORING →
 * INTAKE_LOOP (parallel) → GATE_CHECK → SCORE_ARTIFACTS → PARK → IDLE
 *
 * Features:
 * - VisionPortal for AprilTag/Motif detection
 * - Road Runner for pathing
 * - PIDF launcher controller with voltage compensation
 * - Non-blocking intake while moving (parallel state machine)
 */
@Autonomous(name = "DECODE Autonomous", group = "DECODE")
public class AutonomousDECODE extends LinearOpMode {

    // State Machine Enum
    public enum AutoState {
        INIT,
        DETECT_ZONE,
        SELECT_TRAJECTORY,
        PRELOAD_SCORING,
        INTAKE_LOOP,
        GATE_CHECK,
        SCORE_ARTIFACTS,
        PARK,
        IDLE
    }

    // Hardware
    private HardwarePushbot robot = new HardwarePushbot();
    private MotifDetector vision = new MotifDetector();

    // State management
    private AutoState currentState = AutoState.INIT;
    private AutoState previousState = AutoState.INIT;
    private MotifDetector.DetectedZone targetZone = MotifDetector.DetectedZone.UNKNOWN;

    // Trajectory tracking
    private Trajectory currentTrajectory = null;
    private int intakeCount = 0;
    private static final int MAX_INTAKES = 3;

    // Parallel intake flag
    private AtomicBoolean isIntaking = new AtomicBoolean(false);

    // PIDF Launcher Controller
    private PIDFController launcherPidf = new PIDFController(0.01, 0.0001, 0.001, 0.0001);
    private static final double LAUNCHER_TARGET_VELOCITY = 3000; // RPM

    // Telemetry
    private String statusMessage = "Initializing...";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision.init(hardwareMap, robot.webcam);

        telemetry.addData("Status", "Ready for DECODE Auto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && currentState != AutoState.IDLE) {
            runStateMachine();
            updateParallelIntake();
            telemetry.update();
        }

        vision.close();
    }

    /**
     * Main State Machine Loop
     */
    private void runStateMachine() {
        switch (currentState) {
            case INIT:
                stateInit();
                break;
            case DETECT_ZONE:
                stateDetectZone();
                break;
            case SELECT_TRAJECTORY:
                stateSelectTrajectory();
                break;
            case PRELOAD_SCORING:
                statePreloadScoring();
                break;
            case INTAKE_LOOP:
                stateIntakeLoop();
                break;
            case GATE_CHECK:
                stateGateCheck();
                break;
            case SCORE_ARTIFACTS:
                stateScoreArtifacts();
                break;
            case PARK:
                statePark();
                break;
        }
    }

    /**
     * INIT: Initialize systems
     */
    private void stateInit() {
        statusMessage = "Initializing systems...";

        // Reset launcher
        robot.launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure gate is closed
        robot.closeGate();

        currentState = AutoState.DETECT_ZONE;
    }

    /**
     * DETECT_ZONE: Use vision to detect Motif pattern on Obelisk
     */
    private void stateDetectZone() {
        statusMessage = "Detecting zone...";

        // Run vision detection for a few seconds during initialization
        long detectionStartTime = System.currentTimeMillis();
        int detectionDuration = 3000; // 3 seconds

        while (System.currentTimeMillis() - detectionStartTime < detectionDuration && opModeIsActive()) {
            vision.update();
            telemetry.addData("Vision Tags", vision.getTagCount());
            telemetry.update();
        }

        targetZone = vision.getDetectedZone();

        if (targetZone == MotifDetector.DetectedZone.UNKNOWN) {
            // Default to center if no detection
            targetZone = MotifDetector.DetectedZone.CENTER;
            telemetry.addData("Zone", "Unknown - Defaulting to CENTER");
        } else {
            telemetry.addData("Zone", targetZone.name());
        }

        currentState = AutoState.SELECT_TRAJECTORY;
    }

    /**
     * SELECT_TRAJECTORY: Choose trajectory based on detected zone
     */
    private void stateSelectTrajectory() {
        statusMessage = "Selecting trajectory...";

        // Trajectory selection happens in the next state
        currentState = AutoState.PRELOAD_SCORING;
    }

    /**
     * PRELOAD_SCORING: Score the pre-loaded artifact into the goal
     */
    private void statePreloadScoring() {
        statusMessage = "Scoring pre-load...";

        // Start launcher to score pre-loaded artifact
        double voltage = robot.getBatteryVoltage();
        launcherPidf.setFeedforward(12.0 / voltage * 0.1);
        double launcherPower = launcherPidf.calculate(LAUNCHER_TARGET_VELOCITY, 0);
        robot.startLauncher(launcherPower);

        // Wait for launcher to spin up
        sleep(1000);

        // Stop launcher after scoring
        robot.stopLauncher();

        // Begin parallel intake operation
        isIntaking.set(true);

        currentState = AutoState.INTAKE_LOOP;
    }

    /**
     * INTAKE_LOOP: Non-blocking parallel intake while moving
     */
    private void stateIntakeLoop() {
        statusMessage = "Intaking artifacts: " + intakeCount + "/" + MAX_INTAKES;

        if (intakeCount >= MAX_INTAKES) {
            isIntaking.set(false);
            robot.stopIntake();
            currentState = AutoState.GATE_CHECK;
            return;
        }

        // Intake is handled in parallel by updateParallelIntake()
        // This state allows other operations while intaking

        // Check if we've reached spike mark positions via odometry
        Pose2d currentPose = getCurrentPose();
        if (isAtSpikeMark(currentPose)) {
            intakeCount++;
            sleep(500); // Let intake grab the artifact
        }

        // Continue to next state after sufficient movement time
        // (In real implementation, use odometry or time-based progression)
    }

    /**
     * GATE_CHECK: Check distance sensor for classifier ramp status
     */
    private void stateGateCheck() {
        statusMessage = "Checking classifier ramp...";

        double distance = robot.getRampDistance();
        telemetry.addData("Ramp Distance", distance + " mm");

        // If ramp is full (threshold ~50mm), trigger gate to Secret Tunnel
        if (distance < 50.0) {
            statusMessage = "Ramp full - opening gate to Secret Tunnel";
            robot.openGate();
            sleep(1000); // Let artifacts pass
            robot.closeGate();
        }

        currentState = AutoState.SCORE_ARTIFACTS;
    }

    /**
     * SCORE_ARTIFACTS: Launch collected artifacts into the goal
     */
    private void stateScoreArtifacts() {
        statusMessage = "Scoring collected artifacts...";

        // Spin up launcher
        double voltage = robot.getBatteryVoltage();
        launcherPidf.setFeedforward(12.0 / voltage * 0.1);
        double launcherPower = launcherPidf.calculate(LAUNCHER_TARGET_VELOCITY, 0);
        robot.startLauncher(launcherPower);

        sleep(1500); // Wait for launcher to reach target

        // Release artifacts (open gate briefly)
        robot.openGate();
        sleep(500);
        robot.closeGate();

        // Stop launcher
        robot.stopLauncher();

        currentState = AutoState.PARK;
    }

    /**
     * PARK: Navigate to Base Zone for endgame bonus
     */
    private void statePark() {
        statusMessage = "Parking in Base Zone...";

        // Stop intake if still running
        isIntaking.set(false);
        robot.stopIntake();

        // In actual implementation, this would follow a Road Runner trajectory
        // to the appropriate parking position based on detected zone
        driveToParkPosition();

        statusMessage = "Parked - AUTO Complete";
        currentState = AutoState.IDLE;
    }

    /**
     * Update parallel intake operation (non-blocking)
     */
    private void updateParallelIntake() {
        if (isIntaking.get()) {
            robot.startIntake();
        } else {
            robot.stopIntake();
        }
    }

    /**
     * Get current pose from odometry (placeholder - implement with actual odometry)
     */
    private Pose2d getCurrentPose() {
        // In real implementation, read from Dead Wheel encoders
        return new Pose2d(0, 0, 0);
    }

    /**
     * Check if robot is at a spike mark position
     */
    private boolean isAtSpikeMark(Pose2d pose) {
        // Simplified check - in real implementation use actual coordinates
        Vector2d pos = pose.vec();
        return (Math.abs(pos.getX() + 24) < 5 && Math.abs(pos.getY()) < 5);
    }

    /**
     * Drive to parking position (simplified - replace with Road Runner)
     */
    private void driveToParkPosition() {
        // Drive to base zone based on zone
        switch (targetZone) {
            case LEFT:
                // Drive to left park position
                break;
            case CENTER:
                // Drive to center park position
                break;
            case RIGHT:
                // Drive to right park position
                break;
        }

        // Simple timeout-based movement
        long parkStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - parkStartTime < 3000 && opModeIsActive()) {
            robot.setDrivePower(0.5);
            sleep(100);
        }
        robot.stopDrive();
    }

    /**
     * Simple sleep utility
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * PIDF Controller for Launcher
     * Uses Proportional-Integral-Derivative with Feedforward
     */
    public static class PIDFController {
        private double kP, kI, kD, kF;
        private double target = 0;
        private double previousError = 0;
        private double integral = 0;
        private double feedforward = 0;
        private long lastTime = System.currentTimeMillis();

        public PIDFController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public void setFeedforward(double ff) {
            this.feedforward = ff;
        }

        public void setTarget(double target) {
            this.target = target;
        }

        public double calculate(double currentValue, double dt) {
            // Calculate error
            double error = target - currentValue;

            // Calculate integral (with anti-windup)
            integral += error * dt;
            if (integral > 1.0) integral = 1.0;
            if (integral < -1.0) integral = -1.0;

            // Calculate derivative
            double derivative = (error - previousError) / dt;

            // Calculate output
            double output = kP * error + kI * integral + kD * derivative + kF * feedforward;

            // Clamp output
            output = Math.max(0, Math.min(1.0, output));

            previousError = error;
            return output;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
        }
    }
}