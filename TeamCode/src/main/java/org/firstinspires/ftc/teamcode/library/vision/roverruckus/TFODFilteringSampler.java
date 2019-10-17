package org.firstinspires.ftc.teamcode.library.sampling.roverruckus;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.library.functions.Position;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

class TFODFilteringSampler implements TensorFlowSampler {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final Comparator<Recognition> largestWidthComparator;
    private static final String VUFORIA_KEY = "AVsEI0//////AAABmfbnDsRuiEqQsFcH7Lyqo3QuISqpJopGJxX15CNQ6JRwa6IhhZMdS346pkBRyp94aOkulPOzR+MQZ84lQbPclj/UW9I95nliTUyCT+Ie8Bw9qNN5X5Cv4sBkIAyNJpUXfxMjUf/5Hw098czCJ1HTiVoqUVB+AGNgZ6tLD4AGqbv/ftucrrA/nnzT045vPyCZCKujFStiBc1Hkab9Y96FE5wHPrfBeCrq8nYd0T+mB3eaCO3kUahLaqyjhFAyQnQCpa1oOzsqbDARsc5FuIcACzdzOFSG+LAEOj+Bgb7Nm+jJQHn8rYsCJi94aoSptfsgXSR0N2pTwhM3eQZEf9g6m1fbUJH18eCFJWwS1FVr3U+6";
    private static final Comparator<Recognition> leftmostComparator;
    private static final Predicate<Recognition> squareFilter;
    private static final Predicate<Recognition> onlyGoldMinerals;
    private static final Predicate<Recognition> onlySilverMinerals;
    private double confidence;
    private String output = "no output available";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public TFODFilteringSampler(HardwareMap hardwareMap) throws TFODFilteringSampler.UnsupportedHardwareException {
        // check if tfod is supported
        if (!ClassFactory.getInstance().canCreateTFObjectDetector())
            throw new TensorFlowSampler.UnsupportedHardwareException();
        // init vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // init tfod
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public boolean activate() {
        if (tfod != null) {
            tfod.activate();
            return true;
        }
        else return false;
    }

    public boolean deactivate() {
        if (tfod != null) {
            tfod.deactivate();
            return true;
        } else return false;
    }

    @Override
    public String getOutput() {
        return output;
    }

    @Override
    public FieldSample recognizeOneMineral() {
        output = "";
        confidence = 0.0;
        List<Recognition> tfodRecognitions = tfod.getUpdatedRecognitions();
        if (tfodRecognitions != null) {
            List<Recognition> filteredRecognitions = sort(filter(tfodRecognitions, squareFilter),largestWidthComparator);
            try {
                Recognition largestRecognition = filteredRecognitions.get(0);
                confidence = largestRecognition.getConfidence();
                switch (largestRecognition.getLabel()) {
                    case LABEL_GOLD_MINERAL: return FieldSample.GOLD;
                    case LABEL_SILVER_MINERAL: return FieldSample.SILVER;
                    default: return FieldSample.NULL;
                }
            } catch (IndexOutOfBoundsException | NullPointerException e) {
                return FieldSample.NULL;
            }
        } else return FieldSample.NULL;
    }

    @Override
    public Position recognizeGoldUsingTwoMinerals(Position cameraViewingDirection) throws IllegalArgumentException {
       if (cameraViewingDirection != Position.LEFT & cameraViewingDirection != Position.RIGHT) throw new IllegalArgumentException("Camera viewing direction left and right are only accepted");
        int interpretation = -1;
       output = "Started two-mineral sampling...";
        confidence = 0.0;
       List<Recognition> tfodRecognitions = tfod.getUpdatedRecognitions();
        if (tfodRecognitions != null) {
            List<Recognition> squareMinerals = filter(tfodRecognitions, squareFilter);
            List<Recognition> leftHalfMinerals = sort(filter(squareMinerals, new Predicate<Recognition>() {
                @Override
                public boolean test(Recognition recognition) {
                    return recognition.getLeft() < recognition.getImageWidth() / 2;
                }
            }), largestWidthComparator);
            List<Recognition> rightHalfMinerals = sort(filter(squareMinerals, new Predicate<Recognition>() {
                @Override
                public boolean test(Recognition recognition) {
                    return recognition.getLeft() > recognition.getImageWidth() / 2;
                }
            }), largestWidthComparator);
            try {
                Recognition leftMineral = leftHalfMinerals.get(0);
                Recognition rightMineral = rightHalfMinerals.get(0);
                if (leftMineral.getLabel().equals(LABEL_GOLD_MINERAL) & rightMineral.getLabel().equals(LABEL_SILVER_MINERAL)) interpretation = 0;
                else if (leftMineral.getLabel().equals(LABEL_SILVER_MINERAL) & rightMineral.getLabel().equals(LABEL_GOLD_MINERAL)) interpretation = 1;
                else if (leftMineral.getLabel().equals(LABEL_SILVER_MINERAL) & rightMineral.getLabel().equals(LABEL_SILVER_MINERAL)) interpretation = 2;
                else if (leftMineral.getLabel().equals(LABEL_GOLD_MINERAL) & rightMineral.getLabel().equals(LABEL_GOLD_MINERAL)) interpretation = 3;
                confidence = (leftMineral.getConfidence() + rightMineral.getConfidence()) / 2;

            } catch (IndexOutOfBoundsException e) {
                e.printStackTrace();
                output += "Error: IndexOutOfBoundsException!\n#left: "+leftHalfMinerals.size() + ";\n#right: "+rightHalfMinerals.size()+"\n";
            }
            switch (interpretation) {
                case 0: //left on camera
                    output += "gold on left of frame\n";
                    switch (cameraViewingDirection) {
                        case LEFT: return Position.LEFT;
                        case RIGHT: return Position.CENTER;
                    }
                case 1:
                    output += "gold on right of frame\n";
                    switch (cameraViewingDirection) {
                        case LEFT: return Position.CENTER;
                        case RIGHT: return Position.RIGHT;
                    }
                case 2: //both silver on camera
                    output += "found two silver\n";
                    switch (cameraViewingDirection) {
                        case LEFT: return Position.RIGHT;
                        case RIGHT: return Position.LEFT;
                    }
                default: //both gold on camera
                    output += "both gold or not a match";
                    return Position.NULL;
            }
        } else return Position.NULL;
    }

    @Override
    public Position recognizeGoldUsingThreeMinerals() {
        confidence = 0.0;
        List<Recognition> tfodRecognitions = tfod.getUpdatedRecognitions();
        if (tfodRecognitions != null) {
            List<Recognition> squareMinerals = filter(tfodRecognitions, squareFilter);
            List<Recognition> goldSorted = sort(filter(squareMinerals,onlyGoldMinerals), largestWidthComparator);
            List<Recognition> silverSorted = sort(filter(squareMinerals,onlySilverMinerals), largestWidthComparator);
            try {
                List<Recognition> threeRecognitions = sort(Arrays.asList(goldSorted.get(0),silverSorted.get(0), silverSorted.get(1)), leftmostComparator);
                confidence = (threeRecognitions.get(0).getConfidence() + threeRecognitions.get(1).getConfidence() + threeRecognitions.get(2).getConfidence()) / 3;
                switch (threeRecognitions.indexOf(goldSorted.get(0))) {
                    case 0: return Position.LEFT;
                    case 1: return Position.CENTER;
                    case 2: return Position.RIGHT;
                }
            } catch (IndexOutOfBoundsException e) {
                return Position.NULL;
            }
            return Position.NULL;
        } else return Position.NULL;
    }

    public double getConfidenceOutput() {
        return confidence;
    }

    private List<Recognition> filter(List<Recognition> recognitions, Predicate<Recognition> filter) {
        List<Recognition> newList = new ArrayList<>();
        if (!recognitions.isEmpty()) {
            for (Recognition recognition : recognitions) {
                if (filter.test(recognition)) newList.add(recognition);
            }
        }
        return newList;
    }

    private List<Recognition> sort(List<Recognition> recognitions, Comparator<Recognition> comparator) {
        List<Recognition> newList = new ArrayList<>(recognitions);
        boolean swapRequired = false;
        do {
            swapRequired = false;
            for (int i = 0; i < newList.size(); i++) {
                if (i != newList.size()-1) {
                    int result = comparator.compare(newList.get(i), newList.get(i+1));
                    if (result < 0){
                        Recognition temp = newList.get(i);
                        newList.set(i, newList.get(i+1));
                        newList.set(i+1, temp);
                        swapRequired = true;
                    }
                }
            }
        } while (swapRequired);
        return newList;
    }

    static {
        largestWidthComparator = new Comparator<Recognition>() {
            @Override
            public int compare(Recognition lhs, Recognition rhs) {
                return Float.compare(lhs.getWidth(),rhs.getWidth());
            }
        };
        leftmostComparator = new Comparator<Recognition>() {
            @Override
            public int compare(Recognition lhs, Recognition rhs) {
                return Float.compare(lhs.getLeft(), rhs.getWidth());
            }
        };
        onlyGoldMinerals = new Predicate<Recognition>() {
            @Override
            public boolean test(Recognition recognition) {
                return recognition.getLabel().equals(LABEL_GOLD_MINERAL);
            }
        };
        onlySilverMinerals = new Predicate<Recognition>() {
            @Override
            public boolean test(Recognition recognition) {
                return recognition.getLabel().equals(LABEL_SILVER_MINERAL);
            }
        };
        squareFilter = new Predicate<Recognition>() {
            @Override
            public boolean test(Recognition recognition) {
                float top = recognition.getTop();
                float bottom = recognition.getBottom();
                float height;
                float left = recognition.getLeft();
                float right = recognition.getRight();
                float width;

                if (top > bottom) height = top - bottom;
                else height = bottom - top;
                if (left > right) width = left - right;
                else width = right - left;
                if (width / height < 1.5 & width / height > 0.5) return true;
                else return false;
            }
        };
    }
}
