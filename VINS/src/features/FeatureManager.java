package features;

import java.util.ArrayList;
import java.util.List;

import motionestimation.DevicePose;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.video.Video;

import dlsu.vins.R;
import ekf.PointDouble;
import android.app.Activity;
import android.util.Log;
import android.view.SurfaceView;

public class FeatureManager implements CvCameraViewListener2 {
	private static final String TAG = "Feature Manager";
	private final int frameInterval = 3; // Frames between near and far frame
	
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);

	private OpticalFlow opticalFlow;
	private Triangulation triangulation;
	
	private BaseLoaderCallback loaderCallback;

	private int frames = 0;
	private CameraBridgeViewBase cameraView;
	private boolean framesReady = false;
	
	// Optical flow fields
	private MatOfPoint2f checkpointFeatures;
	private Mat checkpointImage;
	private Mat currentImage;
	
	private List<Mat> images;
	
	private FeatureManagerListener listener;

	public FeatureManager(Activity caller, FeatureManagerListener listener) {
		Log.i(TAG, "constructed");

		Log.i(TAG, "Trying to load OpenCV library");
		this.listener = listener;
		initLoader(caller);

		cameraView = (CameraBridgeViewBase) caller.findViewById(R.id.surface_view);
		// http://stackoverflow.com/a/17872107
		// cameraView.setMaxFrameSize(720, 1280); // sets to 720 x 480
		cameraView.setMaxFrameSize(400, 1280); // sets to 320 x 240
		cameraView.setVisibility(SurfaceView.VISIBLE);

		cameraView.setCvCameraViewListener(this);

		if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, caller, loaderCallback)) {
			Log.e(TAG, "Cannot connect to OpenCV Manager");
		}
	}

	private void initLoader(Activity caller) {
		loaderCallback = new BaseLoaderCallback(caller) {
			@Override
			public void onManagerConnected(int status) {
				switch (status) {
				case LoaderCallbackInterface.SUCCESS: {
					Log.i(TAG, "OpenCV loaded successfully");
					opticalFlow = new OpticalFlow();
					triangulation = new Triangulation();
					cameraView.enableView();
					checkpointFeatures = new MatOfPoint2f();
					checkpointImage = new Mat();
					images = new ArrayList<>();
					listener.initDone();
				} break;
				default: {
					super.onManagerConnected(status);
				} break;
				}
			}
		};
	}

	public void onCameraViewStarted(int width, int height) {}

	public void onCameraViewStopped() {}

	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		Log.d("VINS", "onCameraFrame");
		currentImage = inputFrame.gray();
		return currentImage;
	}

	public FeatureUpdate getFeatureUpdate(DevicePose devicePose) {
		Log.d(TAG, "Getting Feature Update");
		FeatureUpdate update = new FeatureUpdate();
		
		// Delay
		
		if (!framesReady) {
			Mat toAdd = new Mat();
			currentImage.copyTo(toAdd);
			images.add(toAdd);
			if (frames == frameInterval + 3) {
				framesReady = true;
			}
			frames++;
			return update;
		}
		
		// Optical Flow
		
		Mat nearImage = new Mat();
		images.get(0).copyTo(nearImage);
		Mat farImage = new Mat();
		currentImage.copyTo(farImage);
		OpticalFlowResult opflowresult = opticalFlow.getFeatures(checkpointImage, nearImage, farImage, checkpointFeatures);
		update.setBadPointsIndex(opflowresult.getBadPointsIndex());
		opflowresult.getNearFeatures().copyTo(checkpointFeatures);
	
		// Triangulation

		if (opflowresult.isNotEmpty()) {
			MatOfPoint2f nearFeatures = opflowresult.getNearFeatures();
			MatOfPoint2f farFeatures = opflowresult.getFarFeatures();
			double currentSize = opflowresult.getCurrentSize();
			
			TriangulationResult triangulationResult = triangulation.triangulate(devicePose, nearFeatures, farFeatures, currentSize);
			update.setCurrentPoints(triangulationResult.getCurrentPoints());
			update.setNewPoints(triangulationResult.getNewFeatures());
		}
		
		images.add(farImage);
		nearImage.copyTo(checkpointImage);
		images.remove(0);
		frames++;
		
		return update;
	}
}
