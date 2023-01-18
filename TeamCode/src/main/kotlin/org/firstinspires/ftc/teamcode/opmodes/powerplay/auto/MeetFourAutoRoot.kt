package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.powerplay.auto.util.AbstractAutoRoot
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Config
object MeetFourAutoRoot : AbstractAutoRoot() {
    /**
     * Generates a [TrajectorySequence] for a given starting on the field.
     *
     * @param robot The [CheckmateRobot] class passed in from the [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
     * @param getColor A function to get the correct [ColorCone.ConeColor] from the [ColorCone].
     * @param change A lambda to change each position by to re-arrange and re-orient the sequence for each corner.
     * @return The generated [TrajectorySequence].
     */
    override fun gen(
        robot: CheckmateRobot,
        getColor: (ColorCone) -> ColorCone.ConeColor,
        startsLeft: Boolean,
        change: (Pose2d) -> Pose2d
    ): TrajectorySequence {
        if (startsLeft) {
            g_colors.RED.y = og_colors.BLUE.y
            g_colors.BLUE.y = og_colors.RED.y
        }
        else {
            g_colors.RED.y = og_colors.RED.y
            g_colors.BLUE.y = og_colors.BLUE.y
        }

        lateinit var parkPathRed: TrajectorySequence
        lateinit var parkPathGreen: TrajectorySequence
        lateinit var parkPathBlue: TrajectorySequence

        // facing positive x axis
        robot.zelda.poseEstimate = change(a_startPose.toPose2d())
        val fullPath = robot.zelda.trajectorySequenceBuilder(change(a_startPose.toPose2d()))
            // rotate to face 270 (negative y axis)
            .lineToSplineHeading(change(b_pos1.toPose2d()))
            // rotate to face 225 to score in tall junction
            .lineToSplineHeading(change(c_pos2.toPose2d()))
            // score & reset appendages
            /*.addDisplacementMarker {
                robot.liftyLinkage.targetPosition = 1.0
                robot.waitFor { robot.liftyLinkage.currentPosition > .9 }
                robot.currentLinkState = CheckmateRobot.LinkState.CAP
                robot.sleep(3000)
                robot.turret.position = d_capPos.lateralValue
                robot.nightmareSlide.adjustment = d_capPos.forwardValue
                robot.sleep(1000)
                robot.liftyLinkage.targetPosition = .8
                robot.sleep(1500)
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
                robot.sleep(150)
                robot.liftyLinkage.targetPosition = 1.0
                robot.sleep(3000)
                robot.currentLinkState = CheckmateRobot.LinkState.REST
                robot.sleep(2000)
                robot.currentLinkState = CheckmateRobot.LinkState.SNIFF
                robot.sleep(1000)
                robot.liftyLinkage.targetPosition = .0
                robot.sleep(3000)
            }*/
            // face 270 to grab
            .lineToSplineHeading(change(eA_pos3.toPose2d()))
            .lineToSplineHeading(change(eB_pos3.toPose2d()))
            .lineToSplineHeading(change(e_pos3.toPose2d()))
            .addDisplacementMarker {
                robot.sleep(2000)
            }
            .lineToSplineHeading(change(f_pos4.toPose2d()))
            // grab cone
            /*.addDisplacementMarker {
                robot.liftyLinkage.targetPosition = .3
                robot.currentLinkState = CheckmateRobot.LinkState.SNIFF
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
                robot.liftyLinkage.targetPosition = .0
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.CLOSED
                robot.currentLinkState = CheckmateRobot.LinkState.REST
            }*/
            // move to score on medium pole
            .lineToSplineHeading(change(g_pos5.toPose2d()))
            // score & reset appendages
            /*.addDisplacementMarker {
                robot.liftyLinkage.targetPosition = .8
                robot.waitFor { robot.liftyLinkage.currentPosition > .7 }
                robot.currentLinkState = CheckmateRobot.LinkState.CAP
                robot.sleep(300)
                robot.turret.position = d_capPos.lateralValue
                robot.nightmareSlide.adjustment = d_capPos.forwardValue
                robot.sleep(100)
                robot.liftyLinkage.targetPosition = .6
                robot.sleep(150)
                robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
                robot.liftyLinkage.targetPosition = .8
                robot.sleep(300)
                robot.currentLinkState = CheckmateRobot.LinkState.REST
                robot.sleep(300)
                robot.liftyLinkage.targetPosition = .0
            }*/
            // colors
            .addDisplacementMarker {
                robot.zelda.followTrajectorySequenceAsync(
                    when (getColor(robot.colorCone)) {
                        ColorCone.ConeColor.RED -> parkPathRed
                        ColorCone.ConeColor.GREEN -> parkPathGreen
                        ColorCone.ConeColor.BLUE -> parkPathBlue
                    }
                )
            }
            .build()

        parkPathRed = robot.zelda.trajectorySequenceBuilder(fullPath.end())
            .lineToSplineHeading(change(g_colors.RED.toPose2d()))
            .build()
        parkPathGreen = robot.zelda.trajectorySequenceBuilder(fullPath.end())
            .lineToSplineHeading(change(g_colors.GREEN.toPose2d()))
            .build()
        parkPathBlue = robot.zelda.trajectorySequenceBuilder(fullPath.end())
            .lineToSplineHeading(change(g_colors.BLUE.toPose2d()))
            .build()

        return fullPath
    }

    @JvmField var a_startPose = StupidPose(-64.0, -40.0)
    @JvmField var b_pos1 = StupidPose(-36.0, -36.0, 270.0)
    @JvmField var c_pos2 = StupidPose(-36.0, -12.0, 225.0)

    // score & reset appendages
    @JvmField var d_capPos = ExtensionPosition(.5, .5)


    @JvmField var eA_pos3 = StupidPose(-32.0, -36.0, 300.0)
    @JvmField var e_pos3 = StupidPose(-12.0, -60.0, 270.0)
    @JvmField var eB_pos3 = StupidPose(-18.0, -36.0, 320.0)
    // grab cone
    @JvmField var f_pos4 = StupidPose(-12.0, -40.0)
    @JvmField var g_pos5 = StupidPose(-12.0, -36.0, 135.0)
    // score & reset appendages

    @JvmField var og_colors = ForkColor(
        RED = StupidPose(-12.03, -60.03),
        GREEN = StupidPose(-12.03, -36.03),
        BLUE = StupidPose(-12.03, -12.03)
    )

    private var g_colors = ForkColor(
        RED = StupidPose(-12.03, -60.03),
        GREEN = StupidPose(-12.03, -36.03),
        BLUE = StupidPose(-12.03, -12.03)
    )
}