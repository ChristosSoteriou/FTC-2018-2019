package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.General.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.General.Robot.RobotDrive;
import org.firstinspires.ftc.teamcode.General.Robot.RobotLift;

@TeleOp(name = "TeleOpMode", group = "13906")
public class TeleOpMode extends LinearOpMode {
    // Declaring RobotDrive object
    RobotDrive robotDrive = new RobotDrive();
    RobotLift robotLift = new RobotLift();
    RobotArm robotArm = new RobotArm();

    ElapsedTime lastMaxSpeedChange = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Disable Auto Clear
        telemetry.setAutoClear(false);

        Telemetry.Item robotsMaxSpeed = telemetry.addData("Max Robot Speed", 0);
        Telemetry.Item status = telemetry.addData("Status", "Initialized");

        status.setValue("Initialized");
        telemetry.update();

        // Initializing RobotDrive
        robotDrive.init(this, hardwareMap, gamepad1, telemetry, false);
        robotLift.init(hardwareMap, gamepad1, telemetry);
        robotArm.init(hardwareMap, gamepad1, telemetry);

        // Wait Until the start button is pressed on the driver station
        waitForStart();

        status.setValue("Started");
        telemetry.update();

        // Starting up the threads
        robotDrive.start();
        robotLift.start();
        robotArm.start();
        lastMaxSpeedChange.reset();

        // While the program is not stopped
        while (opModeIsActive()) {
            // Check for max speed change
            if (lastMaxSpeedChange.milliseconds() > 10) {
                // Change Robot's drive speed with dpad
                if (gamepad1.dpad_up) {
                    robotDrive.increamentMaxDriveSpeed();
                    lastMaxSpeedChange.reset();
                }
                else if (gamepad1.dpad_down) {
                    robotDrive.decreamentMaxDriveSpeed();
                    lastMaxSpeedChange.reset();
                }
                // Update Max Speed Telemetry
                robotsMaxSpeed.setValue(robotDrive.getMaxDriveSpeed());
            }
            telemetry.update();
        }

        // Kill the threads
        robotDrive.kill();
        robotLift.kill();
        robotArm.kill();
    }
}
