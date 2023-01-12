package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous(name = "Red-Right", preselectTeleOp = "TeleOp")
@Config
class RedRight : BaseOpMode() {
    override fun preRunLoop() {
        robot.zelda.followTrajectorySequenceAsync(
            MeetFourAutoRUtils.gen(
                robot,
                robot.colorCone.leftColor
            ) {
                Pose2d(
                    it.x,
                    it.y,
                    it.heading
                )
            }.build()
        )
    }
}