package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.General.RobotDrive;

@TeleOp(name = "TeleOpMode", group = "#13906")
public class TeleOpMode extends LinearOpMode {
    // Declaring RobotDrive object
    RobotDrive robotDrive;

    @Override
    public void runOpMode() {
        // Initializing RobotDrive
        robotDrive.init(this, hardwareMap, gamepad1);

        // Wait Until the start button is pressed on the driver station
        waitForStart();

        // Starting up the threads
        robotDrive.start();

        // While not stopped
        while (opModeIsActive()) {

        }

        // Kill the threads
        robotDrive.kill();
    }
}
