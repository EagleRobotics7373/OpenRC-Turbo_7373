package org.firstinspires.ftc.teamcode.library.vision.skystone;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import static org.firstinspires.ftc.teamcode.library.vision.skystone.VisionConstants.*;
public class VisionInitializer {
    public static VuforiaLocalizer createVuforia(CameraType cameraType, HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AVsEI0//////AAABmfbnDsRuiEqQsFcH7Lyqo3QuISqpJopGJxX15CNQ6JRwa6IhhZMdS346pkBRyp94aOkulPOzR+MQZ84lQbPclj/UW9I95nliTUyCT+Ie8Bw9qNN5X5Cv4sBkIAyNJpUXfxMjUf/5Hw098czCJ1HTiVoqUVB+AGNgZ6tLD4AGqbv/ftucrrA/nnzT045vPyCZCKujFStiBc1Hkab9Y96FE5wHPrfBeCrq8nYd0T+mB3eaCO3kUahLaqyjhFAyQnQCpa1oOzsqbDARsc5FuIcACzdzOFSG+LAEOj+Bgb7Nm+jJQHn8rYsCJi94aoSptfsgXSR0N2pTwhM3eQZEf9g6m1fbUJH18eCFJWwS1FVr3U+6";
        switch (cameraType) {
            case PHONE_REAR:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
            case PHONE_FRONT:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case WEBCAM:
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        return ClassFactory.getInstance().createVuforia(parameters);
    }

    public static TFObjectDetector attachTFOD(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET_EXT, TFOD_LABEL_STONE, TFOD_LABEL_SKYSTONE);
        return tfod;
    }

    enum CameraType {
        WEBCAM, PHONE_REAR, PHONE_FRONT
    }
}
