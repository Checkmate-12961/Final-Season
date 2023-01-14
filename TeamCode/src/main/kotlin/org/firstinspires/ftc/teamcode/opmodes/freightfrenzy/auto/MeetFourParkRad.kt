package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.MeetFourAutoRUtils.a_startPose
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

@Config
object MeetFourParkRad {
    /**
     * Generates a [TrajectorySequenceBuilder] for a given starting on the field.
     *
     * @param robot The [CheckmateRobot] class passed in from the [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
     * @param color The [ColorCone.ConeColor] representing the rotation of the cone.
     * @param change A lambda to change each position by to re-arrange and re-orient the sequence for each corner.
     * @return The generated [TrajectorySequenceBuilder].
     */
    fun gen(
        robot: CheckmateRobot,
        color: ColorCone.ConeColor,
        startsLeft: Boolean,
        change: (Pose2d) -> Pose2d = { it }
    ): TrajectorySequenceBuilder {
        if (c_colors.RED == oc_colors.RED) {
            if (startsLeft) {
                c_colors.RED = oc_colors.BLUE
                c_colors.BLUE = oc_colors.RED
            }
        }
        else if (!startsLeft) {
            c_colors.RED = oc_colors.RED
            c_colors.BLUE = oc_colors.BLUE
        }
        // facing positive x axis
        robot.zelda.poseEstimate = change(a_startPose.toPose2d())
        return robot.zelda.trajectorySequenceBuilder(change(a_startPose.toPose2d()))
            // move foward to dodge obstacles
            .lineToSplineHeading(change(b_posDodge.toPose2d()))
            // park in correct space
            .lineToSplineHeading(
                change(
                        when (color) {
                            ColorCone.ConeColor.RED -> c_colors.RED
                            ColorCone.ConeColor.GREEN -> c_colors.GREEN
                            ColorCone.ConeColor.BLUE -> c_colors.BLUE
                        }.toPose2d()

                )
            )
    }


    data class StupidPose(
        @JvmField var x: Double = .0,
        @JvmField var y: Double = .0,
        @JvmField var heading: Double = .0
    ) {
        fun toPose2d(): Pose2d = Pose2d(x, y, Math.toRadians(heading))
    }

    data class ForkColor(
        @JvmField var RED: StupidPose = StupidPose(),
        @JvmField var GREEN: StupidPose = StupidPose(),
        @JvmField var BLUE: StupidPose = StupidPose()
    )

    @JvmField var a_startPose = StupidPose(-64.0, -40.0)
    @JvmField var b_posDodge = StupidPose(-36.0, -24.0)


    // Use to change c_colors for different spawnpoints
    // .03 added to avoid possible empty path exceptions
    @JvmField var oc_colors = ForkColor(
        RED = StupidPose(-12.03, -60.03),
        GREEN = StupidPose(-12.03, -36.03),
        BLUE = StupidPose(-12.03, -12.03)
    )

    // Poses for color parking
    @JvmField var c_colors = ForkColor(
        RED = StupidPose(-12.03, -60.03),
        GREEN = StupidPose(-12.03, -36.03),
        BLUE = StupidPose(-12.03, -12.03)
    )
}