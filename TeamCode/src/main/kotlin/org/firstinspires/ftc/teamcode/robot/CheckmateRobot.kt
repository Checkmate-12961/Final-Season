/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.*
import org.firstinspires.ftc.teamcode.robot.util.LynxModuleUtil

/**
 * Checkmate robot class to access subsystems.
 *
 * @constructor
 * Enables Lynx caching. Registers default subsystems.
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
 */
class CheckmateRobot(hardwareMap: HardwareMap) : AbstractRobot() {
    override val tag = "CheckmateRobot"

    /**
     * Access the [Zelda] subsystem.
     */
    val zelda: Zelda get() = subsystems.get<Zelda>()!!

    /**
     * Access the [ColorCone] subsystem.
     */
    val colorCone: ColorCone get() = subsystems.get<ColorCone>()!!

    /**
     * Access the [LiftyLinkage] subsystem.
     */
    val liftyLinkage: LiftyLinkage get() = subsystems.get<LiftyLinkage>()!!

    /**
     * Access the [ClumsyClaw] subsystem.
     */
    val clumsyClaw: ClumsyClaw get() = subsystems.get<ClumsyClaw>()!!

    /**
     * Access the [NightmareSlide] subsystem.
     */
    val nightmareSlide: NightmareSlide get() = subsystems.get<NightmareSlide>()!!

    /**
     * Access the [T] subsystem.
     */
    val turret: T get() = subsystems.get<T>()!!

    /**
     * Puts the thread to sleep.
     *
     * @param milliseconds Duration (milliseconds) to sleep for.
     */
    fun sleep(milliseconds: Long) {
        try {
            Thread.sleep(milliseconds)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }

    /**
     * Busy loops until [callback] is true.
     *
     * @param callback
     */
    fun waitFor(callback: () -> Boolean) { while (!callback()) continue }

    var currentLinkState = LinkState.REST
        set(value) {
            // Checks
            if (currentLinkState != LinkState.REST && value != LinkState.REST) return
            if (value == LinkState.CAP && !this.liftyLinkage.isAboveMid) return

            field = value

            when (value) {
                LinkState.SNIFF -> {
                    this.clumsyClaw.wrist = ClumsyClaw.WristPosition.BIG_EYES
                    this.clumsyClaw.pivot = ClumsyClaw.PivotPosition.GRAB
                    this.turret.locked = false
                    this.nightmareSlide.currentFrame = -1
                    this.liftyLinkage.lockedAboveMid = false
                }
                LinkState.REST -> {
                    this.turret.locked = true
                    if (currentLinkState == LinkState.CAP) {
                        this.nightmareSlide.currentFrame = 2
                        sleep(300)
                        this.nightmareSlide.currentFrame = 1
                        sleep(400)
                    }
                    this.clumsyClaw.gripper = ClumsyClaw.GripperPosition.CLOSED
                    this.clumsyClaw.wrist = ClumsyClaw.WristPosition.SMALL_EYES
                    this.clumsyClaw.pivot = ClumsyClaw.PivotPosition.REST
                    this.nightmareSlide.currentFrame = 0
                    this.liftyLinkage.lockedAboveMid = false
                }
                LinkState.CAP -> {
                    this.clumsyClaw.wrist = ClumsyClaw.WristPosition.SMALL_EYES
                    this.clumsyClaw.pivot = ClumsyClaw.PivotPosition.CAP
                    this.turret.locked = false
                    this.nightmareSlide.currentFrame = 1
                    this.liftyLinkage.lockedAboveMid = true

                    sleep(300)
                    this.nightmareSlide.currentFrame = 2
                    sleep(400)
                    this.nightmareSlide.currentFrame = 3
                }
            }
        }

    enum class LinkState {
        SNIFF, REST, CAP
    }

    init {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Set up the drivetrain
        subsystems.register(Zelda(hardwareMap))

        // Set up the camera (ColorCone)
        subsystems.register(ColorCone(hardwareMap))

        // Set up the lift mechanism
        subsystems.register(LiftyLinkage(hardwareMap))

        // Set up the claw mechanism
        subsystems.register(ClumsyClaw(hardwareMap))

        // Set up the nightmare of a slide mechanism
        subsystems.register(NightmareSlide(hardwareMap))

        // Set up the t
        subsystems.register(T(hardwareMap))
    }
}