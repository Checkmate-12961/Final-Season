package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Disabled
@Autonomous(name = "Blue-Left", preselectTeleOp = "TeleOp")
class BlueLeft : BaseOpMode() {
    override fun preRunLoop() {
        robot.zelda.followTrajectorySequenceAsync(
            MeetFourAutoRUtils.gen(
                robot,
                robot.colorCone.leftColor,
                true,
            ) {
                Pose2d(
                    it.x,
                    -it.y,
                    -it.heading
                )
            }.build()
        )
    }
}