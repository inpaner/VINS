package dlsu.vins;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.KeyPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.KalmanFilter;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;

import android.app.Activity;
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
    private DescriptorExtractor descriptor;
    private DescriptorMatcher matcher;
    
    
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
        
        textBox.setText("wadup");
        
        //createUpdater();
    }
    
    // http://stackoverflow.com/a/7433510
    private void createUpdater() {
        final Handler handler = new Handler();
        Runnable task = new Runnable() {
            @Override
            public void run() {
                updateFPS();
                handler.postDelayed(this, 1000);
            }
        };
        handler.removeCallbacks(task);
        handler.post(task);
    }
    
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
    
    private Mat prevDescriptors;
    private MatOfKeyPoint prevFeatures;
    private Mat prevImage;
    
    Scalar RED = new Scalar(255,0,0);
    Scalar GREEN = new Scalar(0,255,0);
    
    
    // [Feature Detection] http://stackoverflow.com/q/19808296 
    // [Description + Matching] http://stackoverflow.com/a/16107054
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        Log.d("VINS", "onCameraFrame");
        
        // Do not remove out of the loop. Crashes if not re-instantiated every time.
        // why oh why
        detector = FeatureDetector.create(FeatureDetector.ORB);
        descriptor = DescriptorExtractor.create(DescriptorExtractor.ORB);
        matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        // Do not remove
        
        Mat image = inputFrame.gray();
        MatOfKeyPoint features = new MatOfKeyPoint();
        Mat descriptors = new Mat();
        detector.detect(image, features);
        descriptor.compute(image, features, descriptors);
        KeyPoint a = new KeyPoint();
        
        Mat outputImage = image.clone();
        if (prevImage != null) {
            MatOfDMatch matches = new MatOfDMatch(); 
            matcher.match(prevDescriptors, descriptors, matches);
            
            MatOfByte drawnMatches = new MatOfByte();
            
            Log.d("MATCHES", ""+matches.size());
            
            //for (int i = 0; i < matches.rows(); i++) {
            //    Core.circle(gray, features.toList().get(matches.toList().get(i).trainIdx).pt, 8, RED);
            //}
            
            //
            
            //Features2d.drawMatches(prevImage, prevFeatures, image, features, matches, 
            //        outputImg, GREEN, RED, imageOut, Features2d.DRAW_RICH_KEYPOINTS);
        }    
            
        prevDescriptors = descriptors;
        prevFeatures = features;
        prevImage = image;
        
                
        //Imgproc.cvtColor(image, mRgba, Imgproc.COLOR_RGBA2RGB,4);
        //Features2d.drawKeypoints(mRgba, features, mRgba, RED, 3);
        
        
        totalUpdates++;
        return outputImage;
        
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        

        return true;
    }
   
    public boolean onOptionsItemSelected(MenuItem item) {

        return true;
    }
}
