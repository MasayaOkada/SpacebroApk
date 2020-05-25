
package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;

import java.util.ArrayList;
import java.util.List;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import static org.opencv.aruco.Aruco.estimatePoseSingleMarkers;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.judgeSendStart();
        moveToWrapper(10.6, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11, -5.7, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11.5, -5.7, 4.5, 0, 0, 0, 1); //p1-1
        Mat snapshot = api.getMatNavCam();
        String valueX = Convert(snapshot);
        api.judgeSendDiscoveredQR(0, valueX);

        moveToWrapper(11, -6, 5.55, 0, -0.7071068, 0, 0.7071068); //p1-2
        moveToWrapper(11, -5.5, 4.33, 0, -0.7071068, 0, 0.7071068);//p1-3
        moveToWrapper(11, -6.7, 4.33, 0, -0.7071068, 0, 0.7071068);
        moveToWrapper(10.6, -6.7, 4.33, 0, -0.7071068, 0, 0.7071068);
        moveToWrapper(10.6, -7.3, 4.33, 0, -0.7071068, 0, 0.7071068);

        moveToWrapper(10.30, -7.5, 4.7, 0, 0, 1, 0);//p2-1
        moveToWrapper(10.30, -8, 4.7, 0, 0, 1, 0);
        moveToWrapper(11.5, -8, 5, 0, 0, 0, 1);//p2-2
        moveToWrapper(11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068);//p2-3

        api.laserControl(true);
        moveToWrapper(11.1, -6, 5.55, 0, -0.7071068, 0, 0.7071068);

        api.judgeSendFinishSimulation();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }

    }

    private static String Convert(Mat imgs){
        QRCodeDetector detectAndDecode = new QRCodeDetector();
        String value = detectAndDecode.detectAndDecode(imgs);
        return value;
    }

    private void detectMarker(){
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        Mat inputImage = api.getMatNavCam();
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);

       // double cameraMatrix[][] = new double[3][3];
        double cameraMatrix[][] = {{344.173397, 0.000000, 630.793795},{0.000000, 344.277922, 487.033834},{0.000000,
                0.000000, 1.000000}};
        //double distortionCoefficients[][] = new double[1][5];
        double distortionCoefficients[][] = {{-0.152963, 0.017530, -0.001107, -0.000210, 0.000000}};

        Mat rotationMatrix = new Mat(), translationVectors = new Mat(); // 受け取る
        estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors);
    }

    private void estimatePoseSingleMarkers(List<Mat> corners, float v, double[][] cameraMatrix, double[][] distortionCoefficients, Mat rotationMatrix, Mat translationVectors) {
    }

}