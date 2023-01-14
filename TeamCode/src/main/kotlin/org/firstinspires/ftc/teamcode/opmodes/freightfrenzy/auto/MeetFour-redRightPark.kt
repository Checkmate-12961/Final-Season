package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Disabled
@Autonomous(name = "-PARK-Red-Right-", preselectTeleOp = "TeleOp")
class RedRightPark : BaseOpMode() {
    override fun preRunLoop() {
        robot.zelda.followTrajectorySequenceAsync(
            MeetFourParkRad.gen(
                robot,
                robot.colorCone.rightColor,
                false
            ) {
                Pose2d(
                    -it.x,
                    -it.y,
                    kotlin.math.PI + it.heading
                )
            }.build()
        )
    }
}