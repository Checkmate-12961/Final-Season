package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

class MeetFourAutoRUtils {
    fun gen(
        robot: CheckmateRobot,
        change: (Pose2d) -> Pose2d = { it }
    ): TrajectorySequenceBuilder {
        return robot.zelda.trajectorySequenceBuilder(change(startPose.toPose2d()))
    }

    data class StupidPose(@JvmField var x: Double, @JvmField var y: Double, @JvmField var heading: Double) {
        fun toPose2d(): Pose2d = Pose2d(x, y, Math.toRadians(heading))
    }


    companion object {
        var startPose = StupidPose(0.0, 0.0, 0.0)
        var temp1 = StupidPose(0.0, 0.0, 0.0)
        var temp2 = StupidPose(0.0, 0.0, 0.0)
        var temp3 = StupidPose(0.0, 0.0, 0.0)
        var temp4 = StupidPose(0.0, 0.0, 0.0)
        var temp5 = StupidPose(0.0, 0.0, 0.0)
        var temp6 = StupidPose(0.0, 0.0, 0.0)
    }
}