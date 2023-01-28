package org.firstinspires.ftc.teamcode.robot.subsystems.camera

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.value
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

@Config
class ConePositionPipeline : SubsystemOpenCvPipeline() {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val rgb = listOf(Mat(), Mat(), Mat())

    private val greaterByApplied = Mat()
    private val thresholdApplied = Mat()

    private val rAndBGreater = Mat()
    private val sigAndGGreater = Mat()
    private val output = Mat()

    private val nonZeroMasked = Mat()

    @Volatile
    var coneCenter = Point(); private set

    // CONSTANTS //
    private val dotColor = Scalar.all(255.0)

    override fun processFrame(input: Mat): Mat {
        // extract the channels
        rgb.forEachIndexed { i, mat ->
            Core.extractChannel(input, mat, i)
        }

        Core.compare(rgb[allianceColor.coi], Scalar.all(lowerThreshold), thresholdApplied, Core.CMP_GE)
        Core.multiply(thresholdApplied, Scalar.all(greaterBy), greaterByApplied)

        Core.compare(
            greaterByApplied,
            rgb[allianceColor.other().coi],
            rAndBGreater,
            Core.CMP_GT
        )
        Core.compare(
            greaterByApplied,
            rgb[2],
            sigAndGGreater,
            Core.CMP_GT
        )
        Core.bitwise_and(rAndBGreater, sigAndGGreater, output)

        Core.findNonZero(output, nonZeroMasked)

        coneCenter = Core.mean(nonZeroMasked).value.let { Point(it[0], it[1]) }

        Imgproc.circle(
            input,
            coneCenter,
            1,
            dotColor,
            5
        )

        return when (renderMode) {
            RenderMode.INPUT -> input
            RenderMode.OUTPUT -> output
            RenderMode.THRESHOLD_APPLIED -> thresholdApplied
        }
    }

    val coneDirection: Double
        get() = (coneCenter.x - 160) / 160

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("coneDirection", "%.3f", coneDirection)
        telemetry.addData("coneCenter", "(%.1f, %.1f)", coneCenter.x, coneCenter.y)
    }

    enum class RenderMode {
        THRESHOLD_APPLIED, INPUT, OUTPUT
    }

    enum class AllianceColor(val coi: Int, val other: () -> AllianceColor) {
        RED(0, { BLUE }),
        BLUE(2, { RED })
    }

    companion object {
        @JvmField var allianceColor = AllianceColor.BLUE
        @JvmField var lowerThreshold = 180.0
        @JvmField var greaterBy = .9
        @JvmField var renderMode = RenderMode.INPUT
    }
}