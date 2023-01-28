package org.firstinspires.ftc.teamcode.robot.subsystems.camera

import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.opencv.core.Mat

class EmptyPipeline : SubsystemOpenCvPipeline() {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    override fun processFrame(input: Mat?) = input
}