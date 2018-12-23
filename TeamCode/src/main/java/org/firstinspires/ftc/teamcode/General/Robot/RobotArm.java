package org.firstinspires.ftc.teamcode.General.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.General.HelperClasses.ThreadHelper;

public class RobotArm extends ThreadHelper {
    private static int INFINITY = 9999999;
    private static double elbowPower = 1;
    private static double shoulderPower = 1;

    private static int elbowSpeed = 3;
    private static int shoulderSpeed = 2;

    private DcMotor elbow, shoulder;
    private DigitalChannel shoulderLimit;

    private Gamepad gamepad;
    private Telemetry telemetry;

    private int targetElbowPos = 0;
    private int targetShoulderPos = 0;

    private Telemetry.Item elbowPos, shoulderPos, shoulderLimitState;

    public void init(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        elbowPos = telemetry.addData("elbowPos", 0);
        shoulderPos = telemetry.addData("shoulderPos", 0);
        shoulderLimitState = telemetry.addData("Shoulder Limit State", 0);

        // MOTORS
        elbow = hardwareMap.get(DcMotor.class, "elbowMotor");
        shoulder = hardwareMap.get(DcMotor.class, "shoulderMotor");

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(shoulderPower);
        elbow.setPower(elbowPower);

        shoulderLimit = hardwareMap.get(DigitalChannel.class, "shoulderLimit");
        shoulderLimit.setMode(DigitalChannel.Mode.INPUT);

        super.init();
    }

    @Override
    public void loop() {
        elbowPos.setValue(elbow.getCurrentPosition());
        shoulderPos.setValue(shoulder.getCurrentPosition());
        shoulderLimitState.setValue(shoulderLimit.getState());

        elbow.setTargetPosition(targetElbowPos);
        shoulder.setTargetPosition(targetShoulderPos);

        targetElbowPos += gamepad.y ? elbowSpeed : gamepad.a ? -elbowSpeed : 0;
        targetElbowPos = limit(targetElbowPos, 0, 610);

        targetShoulderPos += gamepad.b ? shoulderSpeed : gamepad.x ? -shoulderSpeed : 0;
        targetShoulderPos = limit(targetShoulderPos, 0, 240);
    }

    @Override
    public void kill() {
        super.kill();

        elbow.setPower(0);
        shoulder.setPower(0);
    }

    private int limit(int val, int low, int high) {
        return (val < low ? low : val > high ? high : val);
    }
}
