package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import org.opencv.objdetect.QRCodeDetector;

public class MainActivity extends AppCompatActivity{
 
    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

}

