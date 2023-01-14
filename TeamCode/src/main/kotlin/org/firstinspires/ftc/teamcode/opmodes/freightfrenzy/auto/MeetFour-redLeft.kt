package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Disabled
@Autonomous(name = "Red-Left", preselectTeleOp = "TeleOp")
class RedLeft : BaseOpMode() {
    override fun preRunLoop() {
        robot.zelda.followTrajectorySequenceAsync(
            MeetFourAutoRUtils.gen(
                robot,
                robot.colorCone.leftColor,
                true
            ) {
                Pose2d(
                    -it.x,
                    it.y,
                    Math.PI - it.heading
                )
            }.build()
        )
    }
}