package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@TeleOp(name="gamepadNew", group="Linear Opmode")
//@Disabled
public class gamepadNew extends LinearOpMode {

    //Declaring OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftFrontDrive;
    private DcMotor craneExtend; //Used for encoder wheel forwards
    private DcMotor cranePitch;
    private DcMotor craneElevate;
    private DcMotor fakeMotor; //Used for encoder wheel sideways
    private Servo craneGrab;
    private Servo trayGrab;
    private Servo craneGrabAttitude;
    private Servo flipperLeft;
    private Servo flipperRight;

    public void runOpMode() {
        
//******************************************************************************

        // Initial setup
        // Assign the objects created above to their ports. Hub 1 is the left one.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 0, hub2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1, hub2
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 0, hub1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port !, hub1
        
        // Disabled for now, this is legacy content
        // craneExtend = hardwareMap.get(DcMotor.class, "craneExtend"); //port 0, hub2
        // cranePitch = hardwareMap.get(DcMotor.class, "cranePitch"); //port 1, hub2
        // craneElevate = hardwareMap.get(DcMotor.class, "craneElevate"); //port 2, hub2
        // fakeMotor = hardwareMap.get(DcMotor.class, "fakeMotor"); //port 2, hub3
        // craneGrab = hardwareMap.get(Servo.class, "craneGrab"); //port 0, servo
        // craneGrabAttitude = hardwareMap.get(Servo.class, "craneGrabAttitude"); //port 2, servo
        // trayGrab = hardwareMap.get(Servo.class, "trayGrab"); //port 1, servo
        // flipperLeft = hardwareMap.get(Servo.class, "flipperLeft"); //port 3. servo
        // flipperRight = hardwareMap.get(Servo.class, "flipperRight"); //port 4, servo
        
        // Resets encoders used for the crane. This is an artifact from last year, and we used it for easier testing; in autonomous the
        // crane was "deployed" before it went under the bridge as it had to be tucked into the robot to fit length restrictions.
        // craneElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // cranePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // fakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Sets motors to run to [pulses] encoder pulses
        // craneElevate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // cranePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Testing encoder stuff 
        // leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        
        // Assigns the motors to a forward direction. Positive values in the .setPower() are forwards.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Variable declaration
        double driveX = 0; // Used for diagonal control, positive is forward
        double driveZ = 0; // Used for diagonal control, positive is right
        double driveTurn = 0; // Used for turning controls
        double speed = 0.5;
        double speedTimer = 0;
      
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Done initalizing

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
//******************************************************************************

            // Drive controls \\
            driveX = gamepad1.left_stick_y * speed;
            driveZ = gamepad1.left_stick_x * speed;
            driveTurn = gamepad1.right_stick_x * 0.5;
            
            leftBackDrive.setPower((driveX - driveZ) - driveTurn);
            rightBackDrive.setPower((driveX + driveZ) + driveTurn);
            leftFrontDrive.setPower((driveX + driveZ) - driveTurn);
            rightFrontDrive.setPower((driveX - driveZ) + driveTurn);
            
            // Speed controls \\
            if(gamepad1.left_bumper && speed < 1 && speedTimer < runtime.seconds()) {
                speed+=0.05; speedTimer = runtime.seconds() + 0.5;
            }
            if(gamepad1.right_bumper && speed > 0.1 && speedTimer < runtime.seconds()) {
                speed-=0.05; speedTimer = runtime.seconds() + 0.5;
            }
            
//******************************************************************************

            // Crane controls \\
            //cranePitch.setPower(gamepad2.left_stick_y);
            
//******************************************************************************
            
            // Servo controls \\
            

//******************************************************************************

            // Telemetry display \\
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.addData("Motor Power", "LR (%.2f), FB (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData(" Left Back Encoder ", leftBackDrive.getCurrentPosition());
            telemetry.update();
        }
    
    public class ConceptWebcam extends LinearOpMode {

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    private static final String TAG = "Webcam Sample";

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    /** The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded. */
    private EvictingBlockingQueue<Bitmap> frameQueue;

    /** State regarding where and how to save frames when the 'A' button is pressed. */
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    //----------------------------------------------------------------------------------------------
    // Main OpMode entry
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();
            telemetry.clear();
            telemetry.addData(">", "Started...Press 'A' to capture frame");

            boolean buttonPressSeen = false;
            boolean captureWhenAvailable = false;
            while (opModeIsActive()) {

                boolean buttonIsPressed = gamepad1.a;
                if (buttonIsPressed && !buttonPressSeen) {
                    captureWhenAvailable = true;
                }
                buttonPressSeen = buttonIsPressed;

                if (captureWhenAvailable) {
                    Bitmap bmp = frameQueue.poll();
                    if (bmp != null) {
                        captureWhenAvailable = false;
                        onNewFrame(bmp);
                    }
                }

                telemetry.update();
            }
        } finally {
            closeCamera();
        }
    }

    /** Do something with the frame */
    private void onNewFrame(Bitmap frame) {
        saveBitmap(frame);
        frame.recycle(); // not strictly necessary, but helpful
    }

    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                            new CameraCaptureSession.CaptureCallback() {
                                @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                    /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                     * for the duration of the callback. So we copy here manually. */
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                }
                            },
                            Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                    RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                }
                            })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    private void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            error("exception saving %s", file.getName());
        }
    }
}
}}
