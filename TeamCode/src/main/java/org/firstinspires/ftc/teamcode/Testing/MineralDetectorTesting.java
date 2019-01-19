package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.General.SoftwareTools.MineralDetector;

@TeleOp(name = "MineralDetectorTesting", group = "TESTING")
public class MineralDetectorTesting extends LinearOpMode{
    MineralDetector mineralDetector = new MineralDetector();

    @Override
    public void runOpMode() {
        mineralDetector.init(hardwareMap);

        waitForStart();

        mineralDetector.enable();

        while (opModeIsActive());

        mineralDetector.disable();
    }
}
