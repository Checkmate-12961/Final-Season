package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone

@Autonomous(name = "Blue-Right", group = "FULL", preselectTeleOp = "TeleOp")
class BlueRight : BaseOpMode() {
    override fun setup() {
        robot.zelda!!.followTrajectorySequenceAsync(
            MeetFourAutoRoot.gen(
                robot,
                ColorCone::rightColor,
                false,
            ) {
                Pose2d(
                    it.x,
                    it.y,
                    it.heading
                )
            }
        )
    }
}