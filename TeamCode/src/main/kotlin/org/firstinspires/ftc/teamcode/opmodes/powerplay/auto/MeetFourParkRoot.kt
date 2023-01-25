package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.powerplay.auto.util.AbstractAutoRoot
import org.firstinspires.ftc.teamcode.robot.TheLegend
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

@Config
object MeetFourParkRoot : AbstractAutoRoot() {
    /**
     * Generates a [TrajectorySequenceBuilder] for a given starting on the field.
     *
     * @param robot The [TheLegend] class passed in from the [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
     * @param getColor Callback to get the [ColorCone.ConeColor] representing the rotation of the cone.
     * @param change A lambda to change each position by to re-arrange and re-orient the sequence for each corner.
     * @return The generated [TrajectorySequenceBuilder].
     */
    override fun gen(
        robot: TheLegend,
        getColor: (ColorCone) -> ColorCone.ConeColor,
        startsLeft: Boolean,
        change: (Pose2d) -> Pose2d
    ): TrajectorySequence {
        if (startsLeft) {
            d_colors.RED = od_colors.BLUE
            d_colors.BLUE = od_colors.RED
        }

        else {
            d_colors.RED = od_colors.RED
            d_colors.BLUE = od_colors.BLUE
        }
        // facing positive x axis
        robot.zelda!!.poseEstimate = change(a_startPose.toPose2d())
        return robot.zelda!!.trajectorySequenceBuilder(change(a_startPose.toPose2d()))
            // move to roughly the center of the spawn square
            .lineToSplineHeading(change(b_posCenter.toPose2d()))
            // move foward to dodge obstacles
            .lineToSplineHeading(change(c_posDodge.toPose2d()))
            // park in correct space
            .lineToSplineHeading(
                change(
                        when (getColor(robot.colorCone!!)) {
                            ColorCone.ConeColor.RED -> d_colors.RED
                            ColorCone.ConeColor.GREEN -> d_colors.GREEN
                            ColorCone.ConeColor.BLUE -> d_colors.BLUE
                        }.toPose2d()

                )
            )
            .build()
    }

    @JvmField var a_startPose = StupidPose(-64.0, -40.0)
    @JvmField var b_posCenter = StupidPose(-58.0, -36.0)
    @JvmField var c_posDodge = StupidPose(-36.0, -36.0)

    // Use to change c_colors for different spawnpoints
    // .03 added to avoid possible empty path exceptions
    @JvmField var od_colors = ForkColor(
        RED = StupidPose(-36.03, -12.03),
        GREEN = StupidPose(-36.03, -36.03),
        BLUE = StupidPose(-36.03, -60.03)
    )

    // Poses for color parking
    var d_colors = ForkColor(
        RED = StupidPose(-36.03, -12.03),
        GREEN = StupidPose(-36.03, -36.03),
        BLUE = StupidPose(-36.03, -60.03)
    )
}