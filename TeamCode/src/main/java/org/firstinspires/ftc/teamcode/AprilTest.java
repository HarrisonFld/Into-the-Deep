package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "April Tag Test", group = "Auto")
public class AprilTest extends LinearOpMode
{
    protected DcMotorEx right_drive1;
    protected DcMotorEx right_drive2;
    protected DcMotorEx left_drive1;
    protected DcMotorEx left_drive2;
    AprilTagProcessor tagProcessor;
    protected void initMotors() {

        right_drive1 = hardwareMap.get(DcMotorEx.class, "right_drive1");
        right_drive2 = hardwareMap.get(DcMotorEx.class, "right_drive2");
        left_drive1 = hardwareMap.get(DcMotorEx.class, "left_drive1");
        left_drive2 = hardwareMap.get(DcMotorEx.class, "left_drive2");

        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);

        right_drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    @Override
    public void runOpMode() {
        initMotors();
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                . setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();
       /* while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);


        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);*/


        waitForStart();
        boolean start = false;
        while (opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0)
            {

                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", this.ftcPose.x, this.ftcPose.y, this.ftcPose.z));
                telemetry.addData("bearing", this.ftcPose.bearing);
                telemetry.addData("yaw", this.ftcPose.yaw);
                telemetry.addData("roll", this.ftcPose.roll);
                telemetry.addData("range", this.ftcPose.range);
                telemetry.addData("pitch", this.ftcPose.pitch);
                telemetry.addData("elevation", this.ftcPose.elevation);
                sleep(2000);
                if(!start && tag != null && tagProcessor != null)
                {
                    start = true;
                    fixBot(1.03f, 22, -4);
                }
                telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", this.ftcPose.x, this.ftcPose.y, this.ftcPose.z));
                telemetry.addData("bearing", this.ftcPose.bearing);
                telemetry.addData("yaw", this.ftcPose.yaw);
                telemetry.addData("roll", this.ftcPose.roll);
                telemetry.addData("range", this.ftcPose.range);
                telemetry.addData("pitch", this.ftcPose.pitch);
                telemetry.addData("elevation", this.ftcPose.elevation);
                //telemetry.update();



            }
        }

    }

    int fixbotran = 0;
    AprilTagPoseFtc ftcPose;

    protected void fixBot(float TargetX, float TargetY, float TargetYaw)
    {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        int PosNeg;
        //test the chat gpt code
        double yawError = TargetYaw - tag.ftcPose.yaw;
        double yawTolerance = 0.01;  // Define a small tolerance for yaw
        double pivotPower;

        while (Math.abs(yawError) > yawTolerance) {
            if (tagProcessor.getDetections().isEmpty()) {
                right_drive1.setPower(0);
                right_drive2.setPower(0);
                left_drive1.setPower(0);
                left_drive2.setPower(0);
                 break;
            }
            tag = tagProcessor.getDetections().get(0);
            this.ftcPose = tag.ftcPose;
            PosNeg = (tag.ftcPose.yaw < 0 ? 1 : -1);
            // Update yaw error
            yawError = TargetYaw - tag.ftcPose.yaw;


//            telemetry
            telemetry.addData("yawError", yawError);
            telemetry.addData("current yaw: ", tag.ftcPose.yaw);
            telemetry.update();

            // Proportional control to smooth the yaw adjustment
            double kP = 1.55;  // Proportional control constant (adjust this for better results)
            pivotPower = (kP * yawError) * PosNeg;

            // Ensure the power stays within motor limits [-1, 1]
            pivotPower = Math.max(-1, Math.min(1, pivotPower));

            // Set motor powers for turning the robot
            right_drive1.setPower(-pivotPower);
            right_drive2.setPower(-pivotPower);
            left_drive1.setPower(pivotPower);
            left_drive2.setPower(pivotPower);

        }
        /*
        double pivot;
        while (Math.abs(TargetYaw-tag.ftcPose.yaw) > 1)
        {
            if(tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
                PosNeg = (tag.ftcPose.yaw < 0) ? 1 : -1;
                pivot = (Math.abs(Math.abs(tag.ftcPose.yaw)-Math.abs(TargetYaw))/90)*PosNeg;
                right_drive1.setPower(-pivot);
                right_drive2.setPower(-pivot);
                left_drive1.setPower(pivot);
                left_drive2.setPower(pivot);
            }
        }*/
        sleep(1500);
//        tag = tagProcessor.getDetections().get(0);
        PosNeg = (this.ftcPose.x > 0) ? 1 : -1;
        double InitialX = this.ftcPose.x;
        double horizontal;
        boolean pitchFlagged  = false;
        while (Math.abs(Math.abs(TargetX)-Math.abs(this.ftcPose.x)) > 0.2)
        {
            if(tagProcessor.getDetections().size() > 0) {
                if (!pitchFlagged) {
                    pitchFlagged = true;
                }
                tag = tagProcessor.getDetections().get(0);
                this.ftcPose = tag.ftcPose;
                horizontal = (Math.abs(tag.ftcPose.x / InitialX) * PosNeg) * 0.6;
                right_drive1.setPower(-horizontal);
                right_drive2.setPower(horizontal);
                left_drive1.setPower(horizontal);
                left_drive2.setPower(-horizontal);
            } else {
                if (pitchFlagged) {
                    right_drive1.setPower(0);
                    right_drive2.setPower(0);
                    left_drive1.setPower(0);
                    left_drive2.setPower(0);
                    break;
                }
                horizontal = (Math.abs(this.ftcPose.x / InitialX) * PosNeg) * 0.6;
                right_drive1.setPower(-horizontal);
                right_drive2.setPower(horizontal);
                left_drive1.setPower(horizontal);
                left_drive2.setPower(-horizontal);
            }
        }
        sleep(1500);
//        tag = tagProcessor.getDetections().get(0);
        PosNeg = (TargetY > this.ftcPose.y) ? -1 : 1;
        double InitialY = this.ftcPose.y;
        double vertical;
        boolean rollFlagged = false;
        while (Math.abs(TargetY-this.ftcPose.y) > 0.2)
        {
            if(tagProcessor.getDetections().size() > 0) {
                if (!rollFlagged) {
                    rollFlagged = true;
                }
                tag = tagProcessor.getDetections().get(0);
                this.ftcPose = tag.ftcPose;
                vertical = ((tag.ftcPose.y / InitialY) * PosNeg) * 0.6;
                right_drive1.setPower(vertical);
                right_drive2.setPower(vertical);
                left_drive1.setPower(vertical);
                left_drive2.setPower(vertical);
            } else {
                if (rollFlagged) {
                    right_drive1.setPower(0);
                    right_drive2.setPower(0);
                    left_drive1.setPower(0);
                    left_drive2.setPower(0);
                    break;
                }
                vertical = ((this.ftcPose.y / InitialY) * PosNeg) * 0.6;
                right_drive1.setPower(vertical);
                right_drive2.setPower(vertical);
                left_drive1.setPower(vertical);
                left_drive2.setPower(vertical);
            }
        }
        right_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive1.setPower(0);
        left_drive2.setPower(0);
    }

}
