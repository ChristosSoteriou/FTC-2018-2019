package org.firstinspires.ftc.teamcode.General.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.General.HelperClasses.ThreadHelper;

public class RobotDrive extends ThreadHelper {
    public enum Direction {
        LEFT, RIGHT
    }

    public static final float UNCHANGED = -999;
    private static final double P_DRIVE_COEFF = 0.1;
    private static final double P_TURN_COEFF = 0.02;
    private static final double I_TURN_COEFF = 0;
    private static final double D_TURN_COEFF = 0;

    private Gamepad gamepad;
    private HardwareMap hardwareMap = null;
    public DcMotor left_drive, right_drive;

    private boolean encoderInMove = false;

    private int desired_en_left = 0;
    private int desired_en_right = 0;

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

        super.init();
    }

    @Override
    public void loop() {
        float drive = gamepad.left_stick_y;
        float turn  = -gamepad.left_stick_x;
        float micro_turning = -gamepad.right_stick_x/2;
        turn = turn + micro_turning;
        move(drive + turn, drive - turn);
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
    public void incrementMotorPosition (int l, int r, double power, boolean waitForAction) {
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
}

