package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Config
@Autonomous(name = "schmoove", preselectTeleOp = "TeleOp")
class Schmoove : BaseOpMode() {
    @JvmField
    var schmooveLength = 750L

    /**
     * when you you you you when you run button
     */
    override fun preRunLoop() {
        // go
        robot.drivetrain.setWeightedDrivePower(Pose2d(1.0, 0.0, 0.0))
        // wait
        sleep(schmooveLength)
        // stop going
        robot.drivetrain.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }
}