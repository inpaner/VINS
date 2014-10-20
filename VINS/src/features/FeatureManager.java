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

	private BaseLoaderCallback loaderCallback;
	private FeatureDetector detector;

	private int frames = 0;
	private CameraBridgeViewBase cameraView;
	private boolean framesReady = false;
	
	// Optical flow fields
	private MatOfPoint2f checkpointFeatures;
	private Mat checkpointImage;
	private Mat currentImage;
	
	private List<Mat> images;
	
	// Triangulation fields
	private Size imageSize;
	private Mat cameraMatrix, distCoeffs, Rot, T;
	private Mat R1, R2, P1, P2, Q;
	private Mat points4D;
	private Mat F, E, W;
	private Mat u, w, vt;
	private Mat nullMatF, tempMat, RotW, RotFinal;
	
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
					cameraView.enableView();
					checkpointFeatures = new MatOfPoint2f();
					checkpointImage = new Mat();
					images = new ArrayList<>();
					detector = FeatureDetector.create(FeatureDetector.FAST);
					listener.initDone();
				} break;
				default: {
					super.onManagerConnected(status);
				} break;
				}
			}
		};
	}

	private void initRectifyVariables() {
		// INITIALIZATION FOR STEREORECTIFY()

		// INPUT VARIABLES

		cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
		distCoeffs = Mat.zeros(5, 1, CvType.CV_64F);
		imageSize = new Size(1920, 1080);
		Rot = Mat.zeros(3, 3, CvType.CV_64F);
		T = Mat.ones(3, 1, CvType.CV_64F);

		// CALIBRATION RESULTS FOR 320 x 240
		cameraMatrix.put(0, 0, 287.484405747163, 0, 159.5);
		cameraMatrix.put(1, 0, 0, 287.484405747163, 119.5);
		cameraMatrix.put(2, 0, 0, 0, 1);

		distCoeffs.put(0, 0, 0.1831508618865668);
		distCoeffs.put(1, 0, -0.8391135375141514);
		distCoeffs.put(2, 0, 0);
		distCoeffs.put(3, 0, 0);
		distCoeffs.put(4, 0, 1.067914298622483);

		Rot.put(0, 0, 1, 0, 0);
		Rot.put(1, 0, 0, 1, 0);
		Rot.put(2, 0, 0, 0, 1);

		// OUTPUT VARIABLES

		R1 = Mat.zeros(3, 3, CvType.CV_64F);
		R2 = Mat.zeros(3, 3, CvType.CV_64F);
		P1 = Mat.zeros(3, 4, CvType.CV_64F);
		P2 = Mat.zeros(3, 4, CvType.CV_64F);
		Q = Mat.zeros(4, 4, CvType.CV_64F);

		// INITIALIZATION END

		// CALL STEREORECTIFY EACH FRAME AFTER THE FIRST
		// JUST PASS A NEW ROTATION AND TRANSLATION MATRIX

		// CALIBRATION RESULTS FOR 1920 x 1080
		// cameraMatrix.put(0, 0, 1768.104971372035, 0, 959.5);
		// cameraMatrix.put(1, 0, 0, 1768.104971372035, 539.5);
		// cameraMatrix.put(2, 0, 0, 0, 1);
		//
		// distCoeffs.put(0, 0, 0.1880897270445046);
		// distCoeffs.put(1, 0, -0.7348187497379466);
		// distCoeffs.put(2, 0, 0);
		// distCoeffs.put(3, 0, 0);
		// distCoeffs.put(4, 0, 0.6936210153459164);
	}

	public void onCameraViewStarted(int width, int height) {
		nullMatF = Mat.zeros(0, 0, CvType.CV_64F);
		initRectifyVariables();
	}

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
			images.add(currentImage);
			if (frames == frameInterval + 3) {
				framesReady = true;
			}
			frames++;
			return update;
		}
		
		////// Optical Flow
		
		Mat nearImage = images.get(0);
		Mat farImage = currentImage;
		
		OpticalFlow opticalFlow = new OpticalFlow();
		OpticalFlowResult opflowresult = opticalFlow.getFeatures(checkpointImage, nearImage, farImage, checkpointFeatures);
		
		////// Triangulation

		// TODO: might want to initialize points4D with a large Nx4 Array
		// so that both memory and time will be saved (instead of
		// reallocation each time)
		// TODO: consider separating triangulation into different class

		if (opflowresult.isNotEmpty()) {
			MatOfPoint2f goodNearFeatures = opflowresult.getNearFeatures();
			MatOfPoint2f goodFarFeatures = opflowresult.getFarFeatures();
			double currentSize = opflowresult.getCurrentSize();
			// Solving for Rotation and Translation Matrices

			// Obtaining the Fundamental Matrix

			F = Calib3d.findFundamentalMat(goodNearFeatures, goodFarFeatures);

			cameraMatrix = cameraMatrix.clone();

			tempMat = nullMatF.clone();
			E = nullMatF.clone();

			// Obtaining the Essential Matrix

			Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
			Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

			W = Mat.zeros(3, 3, CvType.CV_64F);
			W.put(0, 1, -1);
			W.put(1, 0, 1);
			W.put(2, 2, 1);
			u = nullMatF.clone();
			w = nullMatF.clone();
			vt = nullMatF.clone();

			// Decomposing Essential Matrix to obtain Rotation and Translation Matrices

			Core.SVDecomp(E, w, u, vt);

			Core.gemm(u, W, 1, nullMatF, 0, tempMat);
			Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot);
			T = u.col(2);

			// (DEBUG) LOGGING THE VARIOUS MATRICES

			// Log.i("E", E.dump());
			// Log.i("K", cameraMatrix.dump());
			// Log.i("F", F.dump());
			// Log.i("K", cameraMatrix.dump());
			// Log.i("E", E.dump());
			// Log.i("w", w.dump());
			// Log.i("u", u.dump());
			// Log.i("vt", vt.dump());
			// Log.i("r", Rot.dump());
			// Log.i("t", T.dump());
			// Log.i("nullMatF", nullMatF.dump());

			RotW = Mat.zeros(3, 3, CvType.CV_64F);
			RotW.put(0, 0, devicePose.getRotWorld().getArray()[0]);
			RotW.put(1, 0, devicePose.getRotWorld().getArray()[1]);
			RotW.put(2, 0, devicePose.getRotWorld().getArray()[2]);
			RotFinal = Mat.zeros(3, 3, CvType.CV_64F);
			Core.gemm(RotW, Rot, 1, Mat.zeros(0, 0, CvType.CV_64F), 0, RotFinal);

			points4D = Mat.zeros(1, 4, CvType.CV_64F);
			Calib3d.stereoRectify(cameraMatrix, distCoeffs, cameraMatrix.clone(), distCoeffs.clone(), 
					imageSize, RotFinal, T, R1, R2, P1, P2, Q);
			Calib3d.triangulatePoints(P1, P2, goodNearFeatures, goodFarFeatures, points4D);
			
			double transPixel[] = T.t().get(0, 0);
			double transMetric[] = {devicePose.get_xPos(), devicePose.get_yPos(), devicePose.get_zPos()};
			double metricScale = 0;
			
			for(int i = 0; i < transPixel.length; ++i)
				metricScale += transMetric[i]/transPixel[i];
			metricScale /= 3;

			// TODO: maybe this method is more optimized??
			// Mat points3D = new Mat();
			// Calib3d.convertPointsFromHomogeneous(points4D, points3D);				
			
			// Log.i(TAG, "points4D size: " + points4D.size().width);
			// Log.i(TAG, T.dump());
			// Log.i(TAG, devicePose.toString());

			List<PointDouble> current2d = new ArrayList<>();
			List<PointDouble> new2d = new ArrayList<>();
			for (int i = 0; i < goodNearFeatures.height(); i++) {
				double x = points4D.get(0, i)[0] * metricScale / points4D.get(3, i)[0];
				double y = points4D.get(1, i)[0] * metricScale / points4D.get(3, i)[0];

				PointDouble point = new PointDouble(x, y);
				if (i < currentSize) {
					current2d.add(point);
				} else {
					new2d.add(point);
				}
			}
			update.setCurrentPoints(current2d);
			update.setNewPoints(new2d);
		}
		update.setBadPointsIndex(opflowresult.getBadPointsIndex());
		opflowresult.getNearFeatures().copyTo(checkpointFeatures);
	
		images.add(farImage);
		nearImage.copyTo(checkpointImage);
		frames++;
		images.remove(0);
		return update;
	}
	
}
