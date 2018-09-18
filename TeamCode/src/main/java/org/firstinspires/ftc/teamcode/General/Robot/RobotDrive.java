package org.firstinspires.ftc.teamcode.General.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.General.HelperClasses.ThreadHelper;

public class RobotDrive extends ThreadHelper {
    private Gamepad gamepad;
    private HardwareMap hardwareMap = null;
    public DcMotor left_drive, right_drive;

    private boolean encoderInMove = false;

    private int desired_en_left = 0;
    private int desired_en_right = 0;

    // Variables to change the max speed of the robot on the fly
    private double maxDriveSpeed;
    private static double speedStep = 0.01;

    private LinearOpMode opMode;

    public RobotDrive() {}

    public void init(LinearOpMode opMode, HardwareMap hm, Gamepad _gamepad) {
        init(opMode, hm, _gamepad, true);
    }

    // Initialize the process
    public void init(LinearOpMode opMode, HardwareMap hm, Gamepad _gamepad, boolean _encoderInMove) {
        hardwareMap = hm;
        this.opMode = opMode;

        encoderInMove = _encoderInMove;

        gamepad = _gamepad;

        // Initialize Motors
        left_drive = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Setting the direction of the motors
        left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make sure the motor actively resists any external forces
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMaxDriveSpeed(0.7);

        super.init();
    }

    @Override
    public void loop() {
        double drive = gamepad.left_stick_y * maxDriveSpeed;
        double turn  = -gamepad.left_stick_x * maxDriveSpeed;
        move(drive + turn, drive - turn);
    }

    @Override
    public void kill() {
        super.kill();

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    // Move the robot by giving it ONLY motor power
    public void move(double powL, double powR) {
        // For using the motors without the use of the encoders
        if (encoderInMove) {
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (powL != 0 && powR != 0) {
            desired_en_left = left_drive.getCurrentPosition();
            desired_en_right = right_drive.getCurrentPosition();
        }

        // Setting the motors power
        left_drive.setPower(powL);
        right_drive.setPower(powR);
    }

    // Moving the robot by giving it a desired motor position
    public void increamentMotorPosition (int l, int r, double power, boolean waitForAction) {
        if (!opMode.isStopRequested()) {
            // For using the build in PID motor control
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            desired_en_left -= l;
            desired_en_right -= r;

            // Set the desire motor position
            left_drive.setTargetPosition(desired_en_left);
            right_drive.setTargetPosition(desired_en_right);

            // Set the power that we want the motor to run at
            left_drive.setPower(power);
            right_drive.setPower(power);

            // Wait for the desired position to be reached
            if (waitForAction) {
                while ((left_drive.isBusy() || right_drive.isBusy()) && !opMode.isStopRequested()) ;
                left_drive.setPower(0);
                right_drive.setPower(0);
            }
        }
    }

    public void setMaxDriveSpeed(double _maxSpeed) {
        maxDriveSpeed = maxDriveSpeed;
    }
    public double getMaxDriveSpeed() {
        return maxDriveSpeed;
    }

    public void increamentMaxDriveSpeed() {
        maxDriveSpeed += speedStep;
        if (maxDriveSpeed > 1) maxDriveSpeed = 1;
    }
    public void decreamentMaxDriveSpeed() {
        maxDriveSpeed -= speedStep;
        if (maxDriveSpeed < 0) maxDriveSpeed = 0;
    }
}

