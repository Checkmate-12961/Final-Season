package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous(name = "-PARK-Blue-Right-", preselectTeleOp = "TeleOp")
@Config
class BlueRightPark : BaseOpMode() {
    override fun preRunLoop() {
        robot.zelda.followTrajectorySequenceAsync(
            MeetFourParkRad.gen(
                robot,
                robot.colorCone.rightColor,
                false,
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