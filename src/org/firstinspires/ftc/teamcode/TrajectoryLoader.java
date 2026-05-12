package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

/**
 * TrajectoryLoader - Road Runner Trajectory Definitions
 *
 * Creates 3 distinct trajectories based on the detected Motif zone:
 * - LEFT: Go to left spike mark, collect artifacts, return to goal, park
 * - CENTER: Go to center spike mark, collect artifacts, return to goal, park
 * - RIGHT: Go to right spike mark, collect artifacts, return to goal, park
 *
 * Each trajectory includes:
 * 1. Pre-load scoring at goal
 * 2. Navigate to spike marks for artifact collection (x3)
 * 3. Gate approach for classifier
 * 4. Park in Base Zone for endgame bonus
 */
public class TrajectoryLoader {

    // Trajectory constraints
    private static final double MAX_VELOCITY = 30.0; // inches per second
    private static final double MAX_ACCELERATION = 40.0; // inches per second^2
    private static final double MAX_ANGULAR_VELOCITY = Math.toRadians(180);
    private static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(240);

    // Field positions (adjust for actual field measurements)
    // Starting position - center of field, facing forward
    private static final Pose2d START_POSE = new Pose2d(0, 0, Math.toRadians(0));

    // Goal positions for scoring
    private static final Pose2d LEFT_GOAL = new Pose2d(-48, 0, Math.toRadians(0));
    private static final Pose2d CENTER_GOAL = new Pose2d(-48, 0, Math.toRadians(0));
    private static final Pose2d RIGHT_GOAL = new Pose2d(-48, 0, Math.toRadians(0));

    // Spike mark positions (where artifacts are collected)
    private static final Vector2d LEFT_SPIKE = new Vector2d(-24, 24);
    private static final Vector2d CENTER_SPIKE = new Vector2d(-24, 0);
    private static final Vector2d RIGHT_SPIKE = new Vector2d(-24, -24);

    // Base zone parking positions
    private static final Pose2d LEFT_PARK = new Pose2d(48, 24, Math.toRadians(0));
    private static final Pose2d CENTER_PARK = new Pose2d(48, 0, Math.toRadians(0));
    private static final Pose2d RIGHT_PARK = new Pose2d(48, -24, Math.toRadians(0));

    /**
     * Build left trajectory (for LEFT zone detection)
     */
    public static Trajectory buildLeftTrajectory(TrajectoryBuilder builder) {
        return builder
                // 1. Score pre-loaded artifact at goal
                .lineTo(LEFT_GOAL.vec())
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)
                // 2. Drive to spike mark #1
                .splineTo(LEFT_SPIKE, Math.toRadians(180))
                .waitSeconds(0.3)
                // 3. Drive to spike mark #2 (back up then forward)
                .splineTo(CENTER_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 4. Drive to spike mark #3
                .splineTo(RIGHT_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 5. Return to goal area
                .splineTo(LEFT_GOAL.vec(), Math.toRadians(180))
                .waitSeconds(0.5)
                // 6. Park in base zone (endgame bonus)
                .splineTo(LEFT_PARK.vec(), Math.toRadians(0))
                .build();
    }

    /**
     * Build center trajectory (for CENTER zone detection)
     */
    public static Trajectory buildCenterTrajectory(TrajectoryBuilder builder) {
        return builder
                // 1. Score pre-loaded artifact at goal
                .lineTo(CENTER_GOAL.vec())
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)
                // 2. Drive to spike mark #1
                .splineTo(LEFT_SPIKE, Math.toRadians(180))
                .waitSeconds(0.3)
                // 3. Drive to spike mark #2
                .splineTo(CENTER_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 4. Drive to spike mark #3
                .splineTo(RIGHT_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 5. Return to goal area
                .splineTo(CENTER_GOAL.vec(), Math.toRadians(180))
                .waitSeconds(0.5)
                // 6. Park in base zone
                .splineTo(CENTER_PARK.vec(), Math.toRadians(0))
                .build();
    }

    /**
     * Build right trajectory (for RIGHT zone detection)
     */
    public static Trajectory buildRightTrajectory(TrajectoryBuilder builder) {
        return builder
                // 1. Score pre-loaded artifact at goal
                .lineTo(RIGHT_GOAL.vec())
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)
                // 2. Drive to spike mark #1
                .splineTo(LEFT_SPIKE, Math.toRadians(180))
                .waitSeconds(0.3)
                // 3. Drive to spike mark #2
                .splineTo(CENTER_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 4. Drive to spike mark #3
                .splineTo(RIGHT_SPIKE, Math.toRadians(0))
                .waitSeconds(0.3)
                // 5. Return to goal area
                .splineTo(RIGHT_GOAL.vec(), Math.toRadians(180))
                .waitSeconds(0.5)
                // 6. Park in base zone
                .splineTo(RIGHT_PARK.vec(), Math.toRadians(0))
                .build();
    }

    /**
     * Create trajectory builder with constraints
     */
    public static TrajectoryBuilder createBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(
                startPose,
                false,
                new MecanumVelocityConstraint(MAX_VELOCITY, 0.5),
                new ProfileAccelerationConstraint(MAX_ACCELERATION)
        );
    }

    /**
     * Simple approach trajectory for pre-load scoring only
     */
    public static Trajectory buildPreloadTrajectory(TrajectoryBuilder builder) {
        return builder
                .lineTo(LEFT_GOAL.vec())
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)
                .build();
    }
}