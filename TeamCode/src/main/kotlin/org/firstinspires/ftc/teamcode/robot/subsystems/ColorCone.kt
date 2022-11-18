package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.ColorConeConstants
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class ColorCone(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "ColorCone"
    override val subsystems = SubsystemMap{ tag }
    private val webcam: OpenCvCamera

    var isStreaming: Boolean? = null
        private set

    // Closes the camera after the initialization period is over
    override fun preLoop() {
        if (isStreaming == true) {
            webcam.stopStreaming()
            isStreaming = false
        } else if (isStreaming == null) {
            isStreaming = false
        }
    }

    val color: ConeColor get() = pipeline.color
    val analysis: List<Int> get() = pipeline.analysis

    // getPosition returns where the barcode is located in a BarcodePosition
    enum class ConeColor(val scalar: Scalar) {
        RED(Scalar(255.0, 0.0, 0.0)),
        GREEN(Scalar(0.0, 255.0, 0.0)),
        BLUE(Scalar(0.0, 0.0, 255.0))
    }

    private val pipeline = ColorConeDeterminationPipeline()

    private class ColorConeDeterminationPipeline : OpenCvPipeline() {
        // channel materials
        private val r = Mat()
        private val g = Mat()
        private val b = Mat()

        // cut out channel materials
        private var boxR = Mat()
        private var boxG = Mat()
        private var boxB = Mat()

        // raw values
        private var redValue = 0
        private var greenValue = 0
        private var blueValue = 0

        // Volatile since accessed by OpMode thread w/o synchronization
        @Volatile
        var color = ConeColor.RED; private set
        val analysis: List<Int> get() = listOf(redValue, greenValue, blueValue)

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

            boxR = r.submat(ColorConeConstants.boxLeft.rectangle)
            boxG = g.submat(ColorConeConstants.boxLeft.rectangle)
            boxB = b.submat(ColorConeConstants.boxLeft.rectangle)
        }

        override fun processFrame(input: Mat): Mat {
            extractChannels(input)

            redValue = Core.mean(boxR).value[0].toInt()
            greenValue = Core.mean(boxG).value[0].toInt()
            blueValue = Core.mean(boxB).value[0].toInt()

            color = if (redValue > greenValue && redValue > blueValue) ConeColor.RED
            else if (greenValue > blueValue) ConeColor.GREEN
            else ConeColor.BLUE

            Imgproc.rectangle(
                input,  // Buffer to draw on
                ColorConeConstants.boxLeft.pointA,  // First point which defines the rectangle
                ColorConeConstants.boxLeft.pointB,  // Second point which defines the rectangle
                color.scalar,  // The color the rectangle is drawn in
                3 // Thickness of indicator rectangle
            )

            return input
        }
    }

    init {
        // Instantiates the webcam "webcam" for OpenCv to use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName::class.java, HardwareNames.Cameras.WEBCAM.id),
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.packageName
            )
        )
        webcam.setPipeline(pipeline)

        // listens for when the camera is opened
        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                if (isStreaming == null) {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
                    isStreaming = true
                    // Streams camera output to the FTCDashboard
                    FtcDashboard.getInstance().startCameraStream(webcam, 4.0)
                }
            }

            override fun onError(errorCode: Int) {}
        })
    }
}

val Scalar.value: DoubleArray get() = this.`val`