package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AlignVision extends SubsystemBase {

    private final Limelight3A limelight;
    private final AllianceColor allianceColor;
    private final Telemetry telemetry;
    private AlignMode alignMode = AlignMode.BROAD_SEARCH;

    public AlignVision(final HardwareMap hardwareMap, AllianceColor allianceColor, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        this.allianceColor = allianceColor;
        this.telemetry = telemetry;
        limelight.pipelineSwitch(0);
    }

    public void printOutputTest() {
        //limelight.updatePythonInputs();
        LLResult result = limelight.getLatestResult();
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }
    }

    // public void setMode() {
    //     limelight.pipelineSwitch(alignMode.pipelineNum);
    // }

    // Returns how close we are to a specific color in the current frame.
    // Detects yellow and blue when alliance choise is blue, and yellow and red when red is chosen
    // Values represent the closest color blob to the claw's center.
    // Returns a stupidly large number if no samples that satisfies the requirements are found
    public double getBroadSearchVal() {
        double[] input = {allianceColor.value, AlignMode.BROAD_SEARCH.value, 0, 0, 0, 0, 0, 0};
        limelight.updatePythonInputs(input);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput()[0];
    }

    // Returns the values of the fine search
    public double[] getFineSearchVal() {
        double[] input = {allianceColor.value, AlignMode.FINE_SEARCH.value, 0, 0, 0, 0, 0, 0};
        limelight.updatePythonInputs(input);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }


    public enum AllianceColor {
        RED(0.0),
        BLUE(1.0);

        private final double value;

        AllianceColor(double value) {
            this.value = value;
        }

    }

    public enum AlignMode {
        BROAD_SEARCH(0),
        FINE_SEARCH(1);

        private final int value;

        AlignMode(int value) {
            this.value = value;
        }
    }
}
