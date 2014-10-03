package dlsu.vins;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.video.Video;

import android.app.Activity;
import android.graphics.Canvas;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.TextView;

public class FastActivity extends Activity implements CvCameraViewListener2 {
    private static final String TAG = "Fast Activity";
    private FeatureDetector detector;
    
    private CameraBridgeViewBase mOpenCvCameraView;
    private TextView textBox;
    private int totalUpdates = 0;
    
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    prevFeatures = new MatOfPoint2f();
                    detector = FeatureDetector.create(FeatureDetector.FAST);
                    
                } break;
                default: {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public FastActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.fastlayout);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.surface_view);
        textBox = (TextView) findViewById(R.id.counter);
        
        // http://stackoverflow.com/a/17872107
        //mOpenCvCameraView.setMaxFrameSize(720, 1280); // sets to 720 x 480
        mOpenCvCameraView.setMaxFrameSize(400, 1280); // sets to 320 x 240
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        
        mOpenCvCameraView.setCvCameraViewListener(this);
    }
    
    // http://stackoverflow.com/a/7433510
    
    private void updateFPS() {
        textBox.setText("FPS: " + totalUpdates);
        Log.d("FPS printing", ""+totalUpdates);
        totalUpdates = 0;
    }
    
    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
    }

    public void onCameraViewStopped() {
    }
    
    private MatOfPoint2f prevFeatures;
    private Mat prevImage;
    
    Scalar RED = new Scalar(255,0,0);
    Scalar GREEN = new Scalar(0,255,0);
    Scalar BLACK = new Scalar(0);
    Scalar WHITE = new Scalar(255);
    
    private int frames = 0;
    private int detectInterval = 5;
    
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        Log.d("VINS", "onCameraFrame");
        
        Mat image = inputFrame.gray();
        Mat detectMask = image.clone();
        detectMask.setTo(WHITE);
        
        if (prevFeatures.size().height > 0) {
            MatOfByte status = new MatOfByte();
            MatOfFloat err = new MatOfFloat();
            MatOfPoint2f nextFeatures = new MatOfPoint2f();
            Video.calcOpticalFlowPyrLK(prevImage, image, prevFeatures, nextFeatures, status, err);
            
            List<Point> oldPoints = prevFeatures.toList();
            List<Point> newPoints = nextFeatures.toList();
            List<Point> goodOldList = new ArrayList<>();
            List<Point> goodNewList = new ArrayList<>();
            List<Integer> badPointsIndex = new ArrayList<>();
            
            int i = 0;
            for (Byte item : status.toList()) {
                if (item.intValue() == 1) {
                    goodOldList.add(oldPoints.get(i));
                    goodNewList.add(newPoints.get(i));
                    Core.circle(detectMask, newPoints.get(i), 10, BLACK, -1);
                }
                else {
                    badPointsIndex.add(Integer.valueOf(i));
                }
                i++;
            }
            
            MatOfPoint2f goodOld = new MatOfPoint2f();
            MatOfPoint2f goodNew = new MatOfPoint2f();
            goodOld.fromList(goodOldList);
            goodNew.fromList(goodNewList);
            
            prevFeatures = goodNew;
            Log.d("Prev", goodOld.size() + "");
        }
        
        if (frames % detectInterval == 0) {
            // Do not remove out of the loop. Crashes if not re-instantiated every time.
            
            MatOfKeyPoint featureMat = new MatOfKeyPoint();
            detector.detect(image, featureMat, detectMask);
            if (featureMat.size().height > 0) {
                MatOfPoint2f newFeatures = convert(featureMat);
                prevFeatures.push_back(newFeatures);
            }
        }
        frames++;
        prevImage = image;
        Log.d("Next", prevFeatures.size() + "");
        return image;
    }
    
    private MatOfPoint2f convert(MatOfKeyPoint keyPoints) {
        KeyPoint[] keyPointsArray = keyPoints.toArray();
        Point[] pointsArray = new Point[keyPointsArray.length];
        
        for (int i = 0; i < keyPointsArray.length; i++) {
            pointsArray[i] = (Point) keyPointsArray[i].pt;
        }
        
        return new MatOfPoint2f(pointsArray);
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        

        return true;
    }
   
    public boolean onOptionsItemSelected(MenuItem item) {

        return true;
    }
}
