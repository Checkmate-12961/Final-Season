package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Disabled
@Config
@Autonomous(name = "schmoove", preselectTeleOp = "TeleOp")
class Schmoove : BaseOpMode() {
    /**
     * when you you you you when you run button
     */
    override fun preRunLoop() {
        // go
        robot.zelda!!.setWeightedDrivePower(Pose2d(1.0, 0.0, 0.0))
        // wait
        sleep(schmooveLength)
        // stop going
        robot.zelda!!.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }

    companion object {
        @JvmField var schmooveLength = 750L
    }
}