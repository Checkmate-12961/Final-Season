package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
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
        if (d_colors.RED == od_colors.RED) {
            if (startsLeft) {
                d_colors.RED = od_colors.BLUE
                d_colors.BLUE = od_colors.RED
            }
        }
        else if (!startsLeft) {
            d_colors.RED = od_colors.RED
            d_colors.BLUE = od_colors.BLUE
        }
        // facing positive x axis
        robot.zelda.poseEstimate = change(a_startPose.toPose2d())
        return robot.zelda.trajectorySequenceBuilder(change(a_startPose.toPose2d()))
            // move to roughly the center of the spawn square
            .lineToSplineHeading(change(b_posCenter.toPose2d()))
            // move foward to dodge obstacles
            .lineToSplineHeading(change(c_posDodge.toPose2d()))
            // park in correct space
            .lineToSplineHeading(
                change(
                        when (color) {
                            ColorCone.ConeColor.RED -> d_colors.RED
                            ColorCone.ConeColor.GREEN -> d_colors.GREEN
                            ColorCone.ConeColor.BLUE -> d_colors.BLUE
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
    @JvmField var b_posCenter = StupidPose(-58.0, -36.0)
    @JvmField var c_posDodge = StupidPose(-36.0, -36.0)


    // Use to change c_colors for different spawnpoints
    // .03 added to avoid possible empty path exceptions
    @JvmField var od_colors = ForkColor(
        RED = StupidPose(-36.03, -60.03),
        GREEN = StupidPose(-36.03, -36.03),
        BLUE = StupidPose(-36.03, -12.03)
    )

    // Poses for color parking
    @JvmField var d_colors = ForkColor(
        RED = StupidPose(-36.03, -60.03),
        GREEN = StupidPose(-36.03, -36.03),
        BLUE = StupidPose(-36.03, -12.03)
    )
}