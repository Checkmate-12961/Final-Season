package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.ClumsyClaw
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

@Config
object MeetFourAutoRUtils {
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
        if (g_colors.RED == og_colors.RED) {
            if (startsLeft) {
                g_colors.RED = og_colors.GREEN
                g_colors.GREEN = og_colors.RED
            }
        }
        else if (!startsLeft) {
            g_colors.RED = og_colors.RED
            g_colors.GREEN = og_colors.GREEN
        }
        // facing positive x axis
        return robot.zelda.trajectorySequenceBuilder(change(a_startPose.toPose2d()))
            // rotate to face 270 (negative y axis)
            .lineToSplineHeading(change(b_pos1.toPose2d()))
            // rotate to face 225 to score in tall junction
            .lineToSplineHeading(change(c_pos2.toPose2d()))
            // score & reset appendages
            .addDisplacementMarker {
                robot.liftyLinkage.targetPosition = 1.0
                robot.waitFor { robot.liftyLinkage.currentPosition > .9 }
                robot.currentLinkState = CheckmateRobot.LinkState.CAP
                robot.sleep(300)
                robot.t.position = d_capPos.lateralValue
                robot.nightmareSlide.adjustment = d_capPos.forwardValue
                robot.sleep(100)
                robot.liftyLinkage.targetPosition = .8
                robot.sleep(150)
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
                robot.liftyLinkage.targetPosition = 1.0
                robot.sleep(300)
                robot.currentLinkState = CheckmateRobot.LinkState.REST
                robot.sleep(300)
                robot.liftyLinkage.targetPosition = .0
            }
            // face 270 to grab
            .lineToSplineHeading(change(e_pos3.toPose2d()))
            // grab cone
            .addDisplacementMarker {
                robot.liftyLinkage.targetPosition = .3
                robot.currentLinkState = CheckmateRobot.LinkState.SNIFF
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
                robot.liftyLinkage.targetPosition = .0
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.CLOSED
                robot.currentLinkState = CheckmateRobot.LinkState.REST
            }
            // move to score on medium pole
            .lineToSplineHeading(change(f_pos4.toPose2d()))
            // score & reset appendages
            .addDisplacementMarker {

            }
            // colors
            .lineToSplineHeading(
                change(
                        when (color) {
                            ColorCone.ConeColor.RED -> g_colors.RED
                            ColorCone.ConeColor.GREEN -> g_colors.GREEN
                            ColorCone.ConeColor.BLUE -> g_colors.BLUE
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

    data class ExtensionPosition(
        @JvmField var forwardValue: Double = .5,
        @JvmField var lateralValue: Double = .5
    )

    @JvmField var a_startPose = StupidPose(-60.0, -36.0)
    @JvmField var b_pos1 = StupidPose(-36.0, -36.0, 270.0)
    @JvmField var c_pos2 = StupidPose(-36.0, -12.0, 225.0)

    // score & reset appendages
    @JvmField var d_capPos = ExtensionPosition(.5, .5)

    @JvmField var e_pos3 = StupidPose(-12.0, -60.0, 270.0)
    // grab cone
    @JvmField var f_pos4 = StupidPose(-12.0, -48.0)
    // score & reset appendages

    @JvmField var og_colors = ForkColor(
        RED = StupidPose(-12.0, -60.0),
        GREEN = StupidPose(-12.0, -36.0),
        BLUE = StupidPose(-12.0, -12.0)
    )

    var g_colors = ForkColor(
        RED = StupidPose(-12.0, -60.0),
        GREEN = StupidPose(-12.0, -36.0),
        BLUE = StupidPose(-12.0, -12.0)
    )
}