package features;

import java.util.ArrayList;
import java.util.List;

import motionestimation.DevicePose;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;

import ekf.PointDouble;

public class Triangulation {
	private Size imageSize;
	private Mat cameraMatrix, distCoeffs, Rot, T;
	private Mat R1, R2, P1, P2, Q;
	private Mat points4D;
	private Mat F, E, W;
	private Mat u, w, vt;
	private Mat nullMatF, tempMat, RotW, RotFinal;

	Triangulation() {
		nullMatF = Mat.zeros(0, 0, CvType.CV_64F);
		initRectifyVariables();
	}

	private void initRectifyVariables() {
		// INITIALIZATION FOR STEREORECTIFY()

		// Input VARIABLES

		cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
		distCoeffs = Mat.zeros(5, 1, CvType.CV_64F);
		imageSize = new Size(240, 320);
		Rot = Mat.zeros(3, 3, CvType.CV_64F);
		T = Mat.ones(3, 1, CvType.CV_64F);

		init240x320();

		// Output Variables

		R1 = Mat.zeros(3, 3, CvType.CV_64F);
		R2 = Mat.zeros(3, 3, CvType.CV_64F);
		P1 = Mat.zeros(3, 4, CvType.CV_64F);
		P2 = Mat.zeros(3, 4, CvType.CV_64F);
		Q = Mat.zeros(4, 4, CvType.CV_64F);
	}

	private void init240x320() {
		cameraMatrix.put(0, 0, 287.484405747163, 0, 119.5);
		cameraMatrix.put(1, 0, 0, 287.484405747163, 159.5);
		cameraMatrix.put(2, 0, 0, 0, 1);

		distCoeffs.put(0, 0, 0.1831508618865668);
		distCoeffs.put(1, 0, -0.8391135375141514);
		distCoeffs.put(2, 0, 0);
		distCoeffs.put(3, 0, 0);
		distCoeffs.put(4, 0, 1.067914298622483);
	}

	private void init1080x1920() {
		cameraMatrix.put(0, 0, 1768.104971372035, 0, 539.5);
		cameraMatrix.put(1, 0, 0, 1768.104971372035, 959.5);
		cameraMatrix.put(2, 0, 0, 0, 1);

		distCoeffs.put(0, 0, 0.1880897270445046);
		distCoeffs.put(1, 0, -0.7348187497379466);
		distCoeffs.put(2, 0, 0);
		distCoeffs.put(3, 0, 0);
		distCoeffs.put(4, 0, 0.6936210153459164);
	}

	TriangulationResult triangulate(DevicePose devicePose, MatOfPoint2f leftFeatures, MatOfPoint2f rightFeatures, double currentSize) {
		// TODO: might want to initialize points4D with a large Nx4 Array
		// so that both memory and time will be saved (instead of
		// reallocation each time)

		// Solving for Rotation and Translation Matrices

		// Obtaining the Fundamental Matrix

		F = Calib3d.findFundamentalMat(leftFeatures, rightFeatures);

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

		// Decomposing Essential Matrix to obtain Rotation and Translation
		// Matrices

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
		Calib3d.stereoRectify(cameraMatrix, distCoeffs, cameraMatrix.clone(), distCoeffs.clone(), imageSize, RotFinal, T, R1, R2, P1, P2, Q);
		Calib3d.triangulatePoints(P1, P2, leftFeatures, rightFeatures, points4D);

		double transPixel[] = T.t().get(0, 0);
		double transMetric[] = { devicePose.get_xPos(), devicePose.get_yPos(), devicePose.get_zPos() };
		double metricScale = 0;

		for (int i = 0; i < transPixel.length; ++i)
			metricScale += transMetric[i] / transPixel[i];
		metricScale /= 3;

		// TODO: maybe this method is more optimized??
		// Mat points3D = new Mat();
		// Calib3d.convertPointsFromHomogeneous(points4D, points3D);

		// Log.i(TAG, "points4D size: " + points4D.size().width);
		// Log.i(TAG, T.dump());
		// Log.i(TAG, devicePose.toString());

		List<PointDouble> current2d = new ArrayList<>();
		List<PointDouble> new2d = new ArrayList<>();
		for (int i = 0; i < leftFeatures.height(); i++) {
			double x = points4D.get(0, i)[0] * metricScale / points4D.get(3, i)[0];
			double y = points4D.get(2, i)[0] * metricScale / points4D.get(3, i)[0];

			PointDouble point = new PointDouble(x, y);
			if (i < currentSize) {
				current2d.add(point);
			} else {
				new2d.add(point);
			}
		}

		return new TriangulationResult(current2d, new2d);
	}
}
