package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

class MeetFourAutoRUtils {
    fun gen(
        robot: CheckmateRobot,
        change: (Pose2d) -> Pose2d = { it }
    ): TrajectorySequenceBuilder {
        return robot.zelda.trajectorySequenceBuilder(change(startPose))
    }

    companion object {
        var startPose = Pose2d(0.0, 0.0, 0.0)
        var temp1 = Pose2d(0.0, 0.0, 0.0)
        var temp2 = Pose2d(0.0, 0.0, 0.0)
        var temp3 = Pose2d(0.0, 0.0, 0.0)
        var temp4 = Pose2d(0.0, 0.0, 0.0)
        var temp5 = Pose2d(0.0, 0.0, 0.0)
        var temp6 = Pose2d(0.0, 0.0, 0.0)
    }
}