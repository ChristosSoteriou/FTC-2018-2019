package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.General.RobotDrive;

public class TeleOp extends LinearOpMode {
    RobotDrive robotDrive;

    @Override
    public void runOpMode() {
        robotDrive.init(this, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }

    }
}
