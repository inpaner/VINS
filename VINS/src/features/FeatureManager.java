package features;

import java.util.ArrayList;
import java.util.List;

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
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);

	private BaseLoaderCallback loaderCallback;

	private FeatureDetector detector;

	private int frames = 0; // TODO: ivan sir what is this for, i dunno
	private CameraBridgeViewBase cameraView;

	// Optical flow fields
	private MatOfPoint2f prevCurrent;
	private MatOfPoint2f prevNew;
	private Mat prevImage;
	private Mat currentImage;

	// Triangulation fields
	private Size imageSize;
	private Mat cameraMatrix, distCoeffs, Rot, T;
	private Mat R1, R2, P1, P2, Q;
	private Mat points4D;
	private Mat F, E, W;
	private Mat u, w, vt;
	private Mat nullMatF, tempMat;
	
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
					prevCurrent = new MatOfPoint2f();
					prevNew = new MatOfPoint2f();
					prevImage = new Mat();
					detector = FeatureDetector.create(FeatureDetector.FAST);
					listener.initDone();
				}
					break;
				default: {
					super.onManagerConnected(status);
				}
					break;
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

	public void onCameraViewStopped() {
	}

	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		Log.d("VINS", "onCameraFrame");
		currentImage = inputFrame.gray();
		frames++;
		return currentImage;
	}

	public FeatureUpdate getFeatureUpdate() {
		Log.d(TAG, "Getting Feature Update");

		Mat detectMask = currentImage.clone();
		detectMask.setTo(WHITE);

		FeatureUpdate update = new FeatureUpdate();
		Log.i(TAG, 	"prevCurrent: " + prevCurrent.size() + 
					"\nprevNew: " + prevNew.size());

		if (prevCurrent.size().height + prevNew.size().height > 0) {

			// // Optical Flow

			MatOfByte status = new MatOfByte();
			MatOfFloat err = new MatOfFloat();
			MatOfPoint2f nextFeatures = new MatOfPoint2f();

			int prevCurrentSize = (int) prevCurrent.size().height; // whut
			if (prevNew != null && prevNew.size().height > 0)
				prevCurrent.push_back(prevNew); // combined

			Video.calcOpticalFlowPyrLK(prevImage, currentImage, prevCurrent, nextFeatures, status, err);

			// Use status to filter out good points from bad

			List<Point> oldPoints = prevCurrent.toList();
			List<Point> newPoints = nextFeatures.toList();
			List<Point> goodOldList = new ArrayList<>();
			List<Point> goodNewList = new ArrayList<>();
			List<Integer> badPointsIndex = new ArrayList<>();

			int index = 0;
			int currentSize = 0;
			for (Byte item : status.toList()) {
				if (item.intValue() == 1) {
					if (index < prevCurrentSize)
						currentSize++;
					goodOldList.add(oldPoints.get(index));
					goodNewList.add(newPoints.get(index));
					Core.circle(detectMask, newPoints.get(index), 10, BLACK, -1); // mask
																					// out
																					// during
																					// detection
				} else if (index < prevCurrentSize) { // TODO: double check sir
					badPointsIndex.add(Integer.valueOf(index));
				}
				index++;
			}

			// Convert from List to OpenCV matrix for triangulation

			MatOfPoint2f goodOld = new MatOfPoint2f();
			MatOfPoint2f goodNew = new MatOfPoint2f();
			goodOld.fromList(goodOldList);
			goodNew.fromList(goodNewList);

			// Triangulation

			// TODO: might want to initialize points4D with a large Nx4 Array
			// so that both memory and time will be saved (instead of
			// reallocation each time)
			// TODO: consider separating triangulation into different class

			if (!goodOld.empty() && !goodNew.empty()) {
				// SOLVING FOR THE ROTATION AND TRANSLATION MATRICES

				// GETTING THE FUNDAMENTAL MATRIX

				F = Calib3d.findFundamentalMat(goodOld, goodNew);

				cameraMatrix = cameraMatrix.clone();

				tempMat = nullMatF.clone();
				E = nullMatF.clone();

				// GETTING THE ESSENTIAL MATRIX

				Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
				Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

				W = Mat.zeros(3, 3, CvType.CV_64F);
				W.put(0, 1, -1);
				W.put(1, 0, 1);
				W.put(2, 2, 1);
				u = nullMatF.clone();
				w = nullMatF.clone();
				vt = nullMatF.clone();

				// DECOMPOSING ESSENTIAL MATRIX TO GET THE ROTATION AND
				// TRANSLATION MATRICES

				// Decomposing Essential Matrix to obtain Rotation and
				// Translation Matrices

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

				points4D = Mat.zeros(1, 4, CvType.CV_64F);
				Calib3d.stereoRectify(cameraMatrix, distCoeffs, cameraMatrix.clone(), distCoeffs.clone(), imageSize, Rot, T, R1, R2, P1,
						P2, Q);
				Calib3d.triangulatePoints(P1, P2, goodOld, goodNew, points4D);

				// TODO: maybe this method is more optimized??
				// Mat points3D = new Mat();
				// Calib3d.convertPointsFromHomogeneous(points4D, points3D);
				
				// becomes 2n: n (with method above) + n (iterating to split into current and new
				// I mean, sure, 2n 

				// Split points to current and new PointDouble
				// TODO verify this shit
				// TODO yass corrected

				Log.i(TAG, "points4D size: " + points4D.size().width);

				List<PointDouble> current2d = new ArrayList<>();
				List<PointDouble> new2d = new ArrayList<>();
				for (int i = 0; i < goodOld.height(); i++) {
					double x = points4D.get(0, i)[0] / points4D.get(3, i)[0];
					double y = points4D.get(1, i)[0] / points4D.get(3, i)[0];

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
			update.setBadPointsIndex(badPointsIndex);
			goodNew.copyTo(prevCurrent);
		}

		// Detect new points based on optical flow mask
		MatOfKeyPoint newFeatures = new MatOfKeyPoint();
		detector.detect(currentImage, newFeatures, detectMask);
		if (newFeatures.size().height > 0) {
			prevNew = convert(newFeatures);
		}

		currentImage.copyTo(prevImage);
		return update;
	}

	private MatOfPoint2f convert(MatOfKeyPoint keyPoints) {
		KeyPoint[] keyPointsArray = keyPoints.toArray();
		Point[] pointsArray = new Point[keyPointsArray.length];

		for (int i = 0; i < keyPointsArray.length; i++) {
			pointsArray[i] = (Point) keyPointsArray[i].pt;
		}

		return new MatOfPoint2f(pointsArray);
	}
	
}
