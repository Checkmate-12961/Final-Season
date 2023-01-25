package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.TheLegend
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ClumsyClaw

@Disabled
@Config
@Autonomous(group = "TEST")
class TestCapSequence : BaseOpMode() {
    override fun preRunLoop() {
        robot.liftyLinkage!!.targetPosition = 1.0
        robot.waitFor { robot.liftyLinkage!!.currentPosition > .9 }
        robot.currentLinkState = TheLegend.LinkState.CAP
        robot.sleep(1000)
        robot.turret!!.position = lateralValue
        robot.nightmareSlide!!.adjustment = forwardValue
        robot.sleep(1000)
        robot.liftyLinkage!!.targetPosition = .8
        robot.waitFor { robot.liftyLinkage!!.currentPosition < .85 }
        robot.clumsyClaw!!.gripper = ClumsyClaw.GripperPosition.OPEN
        robot.sleep(1000)
        robot.liftyLinkage!!.targetPosition = 1.0
        robot.waitFor { robot.liftyLinkage!!.currentPosition > .9 }
        robot.clumsyClaw!!.gripper = ClumsyClaw.GripperPosition.CLOSED
        robot.sleep(1000)
        robot.currentLinkState = TheLegend.LinkState.REST
        robot.currentLinkState = TheLegend.LinkState.SNIFF
        robot.nightmareSlide!!.adjustment = 1.0
        robot.sleep(1000)
        robot.liftyLinkage!!.targetPosition = .0
        robot.waitFor { robot.liftyLinkage!!.currentPosition < .3 }
        robot.currentLinkState = TheLegend.LinkState.REST
    }

    companion object {
        @JvmField var lateralValue = .5
        @JvmField var forwardValue = 1.0
    }
}