package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.frames.CRServoFrame;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;

    @TeleOp
    public class Squigle extends LinearOpMode {

        private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());


        private MotorController leftMotor;
        private MotorController rightMotor;
        private DriveTrain drive;

        private HashMap<String, MotorController> nonDriveMotors;

        private Controller controller1;

        @Override
        public void runOpMode() {

            //Initializes the LogController
            LogController.init();

            leftMotor = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "leftMotor")))
                    .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                    .id("leftMotor")
                    .direction(Motor.Direction.REVERSE)
                    .build();
            rightMotor = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "rightMotor")))
                    .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                    .id("rightMotor")
                    .build();

            drive = new DriveTrain.Builder()
                    .motor(rightMotor, DriveTrain.Orientation.RIGHT_FRONT)
                    .motor(leftMotor, DriveTrain.Orientation.RIGHT_BACK)
                    .driveType(DriveTrain.DriveType.MECANUM)
                    .build();

            nonDriveMotors = new HashMap<>();

            nonDriveMotors.put("leftHand",
                    new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "leftHand")))
                            .id("leftHand")
                            .addLogTarget(MotorController.LogTarget.VELOCITY)
                            .ticksPerRevolution(288)
                            .build());

            nonDriveMotors.put("rightHand",
                    new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "rightHand")))
                            .id("rightHand")
                            .addLogTarget(MotorController.LogTarget.VELOCITY)
                            .ticksPerRevolution(288)
                            .build());

            nonDriveMotors.put("turnHead",
                    new MotorController.Builder(new CRServoFrame(hardwareMap.get(CRServo.class, "turnHead")))
                            .id("turnHead")
                            .build());

            controller1 = new Controller(gamepad1, 0.0f, "1",
                    new Controller.Key[] {
                            Controller.Key.LEFT_STICK_X,
                            Controller.Key.LEFT_STICK_Y,
                            Controller.Key.RIGHT_STICK_X,
                            Controller.Key.RIGHT_STICK_Y
                    });

            LogController.logInfo("Waiting for start...");

            waitForStart();
            LogController.logInfo("Starting TeleOp.");
            while (opModeIsActive()) {
                //Logs data from all MotorControllers, the imu, and odometry
                LogController.logData();
                PIDController.update();

                drive.mecanumDrive(0.5 * controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), new Vector(-controller1.analogDeadband(Controller.Key.LEFT_STICK_X), -1 * controller1.analogDeadband(Controller.Key.LEFT_STICK_Y)));

                nonDriveMotors.get("rightHand").setPower(controller1.getButton(Controller.Key.Y) ? 0.2 : controller1.getButton(Controller.Key.B) ? -0.2 : 0.0);

                nonDriveMotors.get("leftHand").setPower(controller1.getButton(Controller.Key.X) ? 0.2 : controller1.getButton(Controller.Key.A) ? -0.2 : 0.0);

                nonDriveMotors.get("turnHead").setPower(0.4 * controller1.getAnalog(Controller.Key.LEFT_TRIGGER));
                nonDriveMotors.get("turnHead").setPower(-0.4 * controller1.getAnalog(Controller.Key.RIGHT_TRIGGER));
            }
            //Closes all logs
            LogController.logInfo("TeleOp complete.");
            LogController.closeLogs();
        }
    }
