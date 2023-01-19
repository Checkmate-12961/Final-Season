package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone

@Autonomous(name = "Red-Left", group = "FULL", preselectTeleOp = "TeleOp")
class RedLeft : BaseOpMode() {
    override fun setup() {
        robot.zelda!!.followTrajectorySequenceAsync(
            MeetFourAutoRoot.gen(
                robot,
                ColorCone::leftColor,
                true
            ) {
                Pose2d(
                    -it.x,
                    it.y,
                    Math.PI - it.heading
                )
            }
        )
    }
}