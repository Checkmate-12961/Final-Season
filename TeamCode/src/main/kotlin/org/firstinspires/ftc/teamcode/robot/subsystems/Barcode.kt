package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.BarcodeConstants
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

abstract class Barcode(hardwareMap: HardwareMap) : AbstractSubsystem {
    private val webcam: OpenCvCamera

    var isStreaming: Boolean? = null
        private set

    // Closes the camera
    override fun preLoop() {
        if (isStreaming == true) {
            webcam.stopStreaming()
            isStreaming = false
        } else if (isStreaming == null) {
            isStreaming = false
        }
    }

    val position: BarcodePosition
        get() = pipeline.position
    val analysis: List<Int>
        get() = pipeline.analysis

    // getPosition returns where the barcode is located in a BarcodePosition
    enum class BarcodePosition {
        LEFT, MIDDLE, RIGHT
    }

    private val pipeline = BarcodeDeterminationPipeline()

    private class BarcodeDeterminationPipeline : OpenCvPipeline() {
        /*
         * Working variables
         * cb is the used to isolate the blue in the feed.
         */
        private var leftBoxCb: Mat? = null
        private var middleBoxCb: Mat? = null
        private val RGB = Mat()
        private val R = Mat()
        private val G = Mat()
        private val B = Mat()
        private var leftValue = 0
        private var middleValue = 0

        // Volatile since accessed by OpMode thread w/o synchronization
        @Volatile
        var position = BarcodePosition.LEFT
            private set

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        private fun inputToR(input: Mat) {
            Core.extractChannel(RGB, R, 0)
        }

        private fun inputToG(input: Mat) {
            Core.extractChannel(RGB, G, 0)
        }

        private fun inputToB(input: Mat) {
            Core.extractChannel(RGB, B, 0)
        }

        override fun init(firstFrame: Mat) {
            inputToR(firstFrame)

            // LEFTBOX_Cb and MIDDLEBOX_Cb are the blue content of their respective boxes.
            boxR = R.submat(BarcodeConstants.box.rectangle)
            boxG = G.submat(BarcodeConstants.box.rectangle)
            boxB = B.submat(BarcodeConstants.box.rectangle)
        }

        override fun processFrame(input: Mat): Mat {
            inputToCb(input)

            // 
            leftValue = Core.mean(leftBoxCb).`val`[0].toInt()
            middleValue = Core.mean(middleBoxCb).`val`[0].toInt()
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.leftBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.leftBox.pointB,  // Second point which defines the rectangle
                PURPLE,  // The color the rectangle is drawn in
                2
            ) // Thickness of the rectangle lines
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.middleBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.middleBox.pointB,  // Second point which defines the rectangle
                PURPLE,  // The color the rectangle is drawn in
                2
            ) // Thickness of the rectangle lines
            position = BarcodePosition.LEFT // Record our analysis
            position = if (leftValue > middleValue && leftValue > BarcodeConstants.leftBox.threshold) {
                BarcodePosition.LEFT
            } else if (middleValue > BarcodeConstants.middleBox.threshold) {
                BarcodePosition.MIDDLE
            } else {
                BarcodePosition.RIGHT
            }
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.leftBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.leftBox.pointB,  // Second point which defines the rectangle
                if (position == BarcodePosition.LEFT) YELLOW else RED,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.middleBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.middleBox.pointB,  // Second point which defines the rectangle
                if (position == BarcodePosition.MIDDLE) YELLOW else BLUE,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
            return input
        }

        val analysis: List<Int>
            get() = listOf(leftValue, middleValue)

        companion object {
            // Some color constants
            // These dictate the color of the boxes when you see the camera output
            private val RED = Scalar(255.0, 0.0, 0.0)
            private val BLUE = Scalar(0.0, 0.0, 255.0)
            private val PURPLE = Scalar(51.0, 12.0, 47.0)
            private val YELLOW = Scalar(255.0, 255.0, 0.0)
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
                    FtcDashboard.getInstance().startCameraStream(webcam, 12.0)
                }
            }

            override fun onError(errorCode: Int) {}
        })
    }
}