package org.firstinspires.ftc.teamcode.General.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.General.HelperClasses.ThreadHelper;

public class RobotLift extends ThreadHelper {
    private static double liftPower = 1;
    private static int liftSpeed = 5;

    DcMotor liftMotor;

    Gamepad gamepad;
    Telemetry telemetry;

    private int targetLiftPos = 0;

    Telemetry.Item liftPos;

    public void init(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        liftPos = telemetry.addData("Lift Position", 0);

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(liftPower);

        super.init();
    }

    @Override
    public void loop() {
        liftPos.setValue(liftMotor.getCurrentPosition());

        liftMotor.setTargetPosition(targetLiftPos);
        targetLiftPos += gamepad.right_bumper ? liftSpeed : gamepad.left_bumper ? -liftSpeed : 0;
        targetLiftPos = limit(targetLiftPos, 0, 827);
    }

    @Override
    public void kill() {
        super.kill();

        liftMotor.setPower(0);
    }

    private int limit(int val, int low, int high) {
        return (val < low ? low : val > high ? high : val);
    }
}

