package org.firstinspires.ftc.teamcode.library.sampling.roverruckus;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.functions.Position;

public interface TensorFlowSampler {
    FieldSample recognizeOneMineral();

    Position recognizeGoldUsingTwoMinerals(Position cameraViewingDirection);

    Position recognizeGoldUsingThreeMinerals();

    boolean activate();

    boolean deactivate();

    String getOutput();

    class SamplingException extends Exception {
        public SamplingException() {
            super();
        }
        public SamplingException(String message) {
            super(message);
        }
    }
    class UnsupportedHardwareException extends SamplingException {
        public UnsupportedHardwareException() {
            super("This Android phone does not support TFOD.");
        }
    }

    class Factory {
        public static TensorFlowSampler newSampler(HardwareMap hardwareMap) throws UnsupportedHardwareException {
            return new TFODFilteringSampler(hardwareMap);
        }
        private Factory() {}
    }
}
