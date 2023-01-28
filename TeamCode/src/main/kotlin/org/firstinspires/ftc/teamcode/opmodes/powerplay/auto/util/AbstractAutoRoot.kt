package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.robot.TheLegend
import org.firstinspires.ftc.teamcode.robot.subsystems.camera.SignalPipeline
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

abstract class AbstractAutoRoot {
    /**
     * Generates a [TrajectorySequenceBuilder] for a given starting on the field.
     *
     * @param robot The [TheLegend] class passed in from the [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
     * @param getColor Callback to get the [SignalPipeline.SignalColor] representing the rotation of the cone.
     * @param change A lambda to change each position by to re-arrange and re-orient the sequence for each corner.
     * @return The generated [TrajectorySequenceBuilder].
     */
    abstract fun gen(
        robot: TheLegend,
        getColor: (SignalPipeline) -> SignalPipeline.SignalColor,
        startsLeft: Boolean,
        change: (Pose2d) -> Pose2d = { it }
    ): TrajectorySequence

    data class StupidPose(
        @JvmField var x: Double = .0,
        @JvmField var y: Double = .0,
        @JvmField var heading: Double = .0
    ) {
        fun toPose2d(): Pose2d = Pose2d(x * (23.5/24.0), y * (23.5/24.0), Math.toRadians(heading))
    }

    data class ForkColor(
        @JvmField var RED: StupidPose = StupidPose(),
        @JvmField var GREEN: StupidPose = StupidPose(),
        @JvmField var BLUE: StupidPose = StupidPose()
    )

    data class ExtensionPosition(
        @JvmField var forwardValue: Double = .5,
        @JvmField var lateralValue: Double = .5
    )
}