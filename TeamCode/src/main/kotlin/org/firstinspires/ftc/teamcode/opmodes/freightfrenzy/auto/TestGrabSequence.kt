package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ClumsyClaw

@Autonomous
@Config
class TestGrabSequence : BaseOpMode() {
    override fun preRunLoop() {
        robot.liftyLinkage.targetPosition = 1.0
        robot.waitFor { robot.liftyLinkage.currentPosition > .9 }
        robot.currentLinkState = CheckmateRobot.LinkState.CAP
        robot.sleep(1000)
        robot.turret.position = lateralValue
        robot.nightmareSlide.adjustment = forwardValue
        robot.sleep(1000)
        robot.liftyLinkage.targetPosition = .8
        robot.sleep(1000)
        robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.OPEN
        robot.liftyLinkage.targetPosition = 1.0
        robot.sleep(1000)
        robot.currentLinkState = CheckmateRobot.LinkState.REST
        robot.currentLinkState = CheckmateRobot.LinkState.SNIFF
        robot.nightmareSlide.adjustment = 1.0
        robot.sleep(1000)
        robot.liftyLinkage.targetPosition = .0
        robot.waitFor { robot.liftyLinkage.currentPosition < .3 }
        robot.currentLinkState = CheckmateRobot.LinkState.REST
    }

    companion object {
        @JvmField var lateralValue = .5
        @JvmField var forwardValue = .5
    }
}