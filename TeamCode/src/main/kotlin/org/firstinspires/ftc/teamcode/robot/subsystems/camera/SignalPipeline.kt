package org.firstinspires.ftc.teamcode.robot.subsystems.camera

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.colorcone.ColorConeConstants
import org.firstinspires.ftc.teamcode.robot.subsystems.value
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class SignalPipeline : SubsystemOpenCvPipeline() {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    // channel materials
    private val r = Mat()
    private val g = Mat()
    private val b = Mat()

    // cut out channel materials
    private var boxRightR = Mat()
    private var boxRightG = Mat()
    private var boxRightB = Mat()

    private var boxLeftR = Mat()
    private var boxLeftG = Mat()
    private var boxLeftB = Mat()

    // raw values
    private var rightRedValue = 0
    private var rightGreenValue = 0
    private var rightBlueValue = 0

    private var leftRedValue = 0
    private var leftGreenValue = 0
    private var leftBlueValue = 0

    // Volatile since accessed by OpMode thread w/o synchronization
    @Volatile
    var rightColor = SignalColor.RED; private set
    private val rightAnalysis: List<Int> get() = listOf(rightRedValue, rightGreenValue, rightBlueValue)

    @Volatile
    var leftColor = SignalColor.RED; private set
    private val leftAnalysis: List<Int> get() = listOf(leftRedValue, leftGreenValue, leftBlueValue)

    /**
     * This function takes the RGB frame and extracts the R, G, and B channels to the variables
     * `r`, `g`, and `b` respectively.
     *
     * @param input the frame to extract from.
     */
    private fun extractChannels(input: Mat) {
        Core.extractChannel(input, r, 0)
        Core.extractChannel(input, g, 1)
        Core.extractChannel(input, b, 2)
    }

    override fun init(firstFrame: Mat) {
        extractChannels(firstFrame)

        boxRightR = r.submat(ColorConeConstants.boxRight.rectangle)
        boxRightG = g.submat(ColorConeConstants.boxRight.rectangle)
        boxRightB = b.submat(ColorConeConstants.boxRight.rectangle)

        boxLeftR = r.submat(ColorConeConstants.boxLeft.rectangle)
        boxLeftG = g.submat(ColorConeConstants.boxLeft.rectangle)
        boxLeftB = b.submat(ColorConeConstants.boxLeft.rectangle)
    }

    override fun processFrame(input: Mat): Mat {
        extractChannels(input)

        rightRedValue = Core.mean(boxRightR).value[0].toInt()
        rightGreenValue = Core.mean(boxRightG).value[0].toInt()
        rightBlueValue = Core.mean(boxRightB).value[0].toInt()

        rightColor = if (rightRedValue > rightGreenValue && rightRedValue > rightBlueValue) SignalColor.RED
        else if (rightGreenValue > rightBlueValue) SignalColor.GREEN
        else SignalColor.BLUE

        leftRedValue = Core.mean(boxLeftR).value[0].toInt()
        leftGreenValue = Core.mean(boxLeftG).value[0].toInt()
        leftBlueValue = Core.mean(boxLeftB).value[0].toInt()

        leftColor = if (leftRedValue > leftGreenValue && leftRedValue > leftBlueValue) SignalColor.RED
        else if (leftGreenValue > leftBlueValue) SignalColor.GREEN
        else SignalColor.BLUE

        Imgproc.rectangle(
            input,  // Buffer to draw on
            ColorConeConstants.boxRight.pointA,  // First point which defines the rectangle
            ColorConeConstants.boxRight.pointB,  // Second point which defines the rectangle
            rightColor.scalar,  // The color the rectangle is drawn in
            3 // Thickness of indicator rectangle
        )
        Imgproc.rectangle(
            input,  // Buffer to draw on
            ColorConeConstants.boxLeft.pointA,  //First point which defines the rectangle
            ColorConeConstants.boxLeft.pointB,  // Second point which defines the rectangle
            leftColor.scalar, // Thickness of the indicator rectangle
            3
        )

        return input
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("colorCone(R)", rightColor)
        telemetry.addData("colorConeRaw(R)", rightAnalysis)

        telemetry.addData("colorCone(L)", leftColor)
        telemetry.addData("colorConeRaw(L)", leftAnalysis)
    }

    // getPosition returns where the barcode is located in a BarcodePosition
    enum class SignalColor(val scalar: Scalar) {
        RED(Scalar(255.0, 0.0, 0.0)),
        GREEN(Scalar(0.0, 255.0, 0.0)),
        BLUE(Scalar(0.0, 0.0, 255.0))
    }
}