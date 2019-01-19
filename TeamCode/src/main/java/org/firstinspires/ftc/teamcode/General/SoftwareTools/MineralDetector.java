package org.firstinspires.ftc.teamcode.General.SoftwareTools;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MineralDetector {
    GoldAlignDetector goldDetector = new GoldAlignDetector();


    public void init(HardwareMap hardwareMap) {
        goldDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldDetector.useDefaults();

        // Optional Tuning
        goldDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldDetector.downscale = 0.4; // How much to downscale the input frames

        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        goldDetector.maxAreaScorer.weight = 0.005;

        goldDetector.ratioScorer.weight = 5;
        goldDetector.ratioScorer.perfectRatio = 1.0;
    }

    public void enable() {
        goldDetector.enable();
    }
    public void disable() {
        goldDetector.disable();
    }




}
