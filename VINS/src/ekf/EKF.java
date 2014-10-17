package ekf;

import java.util.ArrayList;
import java.util.Random;

import motionestimation.DevicePose;
import Jama.Matrix;
import android.util.Log;

public class EKF {

	private ArrayList<Double> X; // State Vector
	private ArrayList<ArrayList<Double>> P; // Covariance Matrix
	private int numFeatures;

	/*
	 * These are jacobians of the prediction model used when adding a new
	 * feature in the covariance matrix. They are updated every INS update
	 * because they are based on displacement and heading.
	 */
	private Matrix jrMatrix;
	private Matrix jzMatrix;

	/* Constants */
	public static final double VRV_VARIANCE = 0.01;
	public static final double P_DIAGONAL_INITIAL = 0.1;
	public static final double Q_NOISE = 0.1;

	public EKF() {
		X = createX();
		P = createP();

		jrMatrix = this.createJRMatrix(0, 0);
		jzMatrix = this.createJZMatrix(0, 0, 0);
	}

	/********** Getters **********/
	public ArrayList<ArrayList<Double>> getP() {
		return (ArrayList<ArrayList<Double>>) P.clone();
	}

	public DevicePose getCurrDevicePose() {
		PointDouble deviceCoords = getDeviceCoords();
		DevicePose pose = new DevicePose(deviceCoords.getX(), deviceCoords.getY(), 0, getHeadingDegrees());
		return pose;
	}

	public ArrayList<Double> getX() {
		return (ArrayList<Double>) X.clone();
	}

	private PointDouble getDeviceCoords() {
		PointDouble point = new PointDouble(X.get(0), X.get(1));
		return point;
	}

	private PointDouble getFeatureCoordsFromStateVector(int featureIndex) {

		int stateVectorIndexOfFeature = 3 + featureIndex * 2;
		double targetFeatureX = X.get(stateVectorIndexOfFeature);
		double targetFeatureY = X.get(stateVectorIndexOfFeature + 1);

		PointDouble point = new PointDouble(targetFeatureX, targetFeatureY);

		return point;
	}

	private double getHeadingDegrees() {
		return Math.toDegrees(this.getHeadingRadians());
	}

	private double getHeadingRadians() {
		return X.get(2);
	}

	/********** INS Update **********/

	// Performs the state update depending on displacement in meters, and
	// heading in degrees
	public void predictFromINS(double displacement, double headingRadians) {

		// Initialization of variables
		double displacementX = displacement * Math.cos(headingRadians);
		double displacementY = displacement * Math.sin(headingRadians);
		double displacementHeading = headingRadians - X.get(2);

		// Update the state vector
		double newX = X.get(0) + displacementX;
		double newY = X.get(1) + displacementY;

		X.set(0, newX);
		X.set(1, newY);
		X.set(2, headingRadians);

		// Update the upper left 3x3 sub-covariance matrix
		Matrix qMatrix = createQ(displacementX, displacementY, displacementHeading);

		Matrix pPhiMatrix = this.extractPPhi();

		Matrix aMatrix = createA(displacementX, displacementY); // Jacobian of
																// Prediction
																// Model

		// pPhi = A * pPphi * A^T + Q
		pPhiMatrix = aMatrix.times(pPhiMatrix).times(aMatrix.transpose()).plus(qMatrix);

		for (int i = 0; i < pPhiMatrix.getRowDimension(); i++)
			for (int j = 0; j < pPhiMatrix.getColumnDimension(); j++)
				P.get(i).set(j, pPhiMatrix.get(i, j));

		// Update the first 3 columns of P (device to feature correlation) P_ri
		// = A * P_ri

		for (int i = 0; i < numFeatures; i++) {
			Matrix PriMatrix = extractPri(i);
			PriMatrix = aMatrix.times(PriMatrix);

			int targetStartRowIndex = 0;
			int targetStartColIndex = 3 + i * 2;

			for (int j = 0; j < PriMatrix.getRowDimension(); j++)
				for (int k = 0; k < PriMatrix.getColumnDimension(); k++)
					P.get(targetStartRowIndex + j).set(targetStartColIndex + k, PriMatrix.get(j, k));

			// Also update the transpose
			Matrix PriMatrixTranspose = PriMatrix.transpose();

			// swap row and col
			int temp = targetStartRowIndex;
			targetStartRowIndex = targetStartColIndex;
			targetStartColIndex = temp;

			for (int j = 0; j < PriMatrixTranspose.getRowDimension(); j++)
				for (int k = 0; k < PriMatrixTranspose.getColumnDimension(); k++)
					P.get(targetStartRowIndex + j).set(targetStartColIndex + k, PriMatrixTranspose.get(j, k));

		}

		// Update Jr and Jz matrices
		jrMatrix = this.createJRMatrix(displacementX, displacementY);
		jzMatrix = this.createJZMatrix(displacementX, displacementY, headingRadians);

	}

	private Matrix extractPri(int index) {
		int startRowIndex = 3 + index * 2;

		return this.extractSubMatrix(0, 2, startRowIndex, startRowIndex + 1);
	}

	private Matrix extractPPhi() {
		return this.extractSubMatrix(0, 2, 0, 2);
	}

	private Matrix extractSubMatrix(int startRow, int endRow, int startCol, int endCol) {
		double[][] sub = new double[endRow - startRow + 1][endCol - startCol + 1];
		for (int i = startRow; i <= endRow; i++)
			for (int j = startCol; j <= endCol; j++)
				sub[i - startRow][j - startCol] = P.get(i).get(j);
		return new Matrix(sub);
	}

	/********** V-INS Update **********/

	// Method for correcting the state vector based on re-observed features.
	public void updateFromReobservedFeature(int featureIndex, double fX, double fY) {

		PointDouble featureCoords = this.getFeatureCoordsFromStateVector(featureIndex);
		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble observedFeatureCoords = new PointDouble(fX, fY);

		double observedDistance = deviceCoords.computeDistanceTo(observedFeatureCoords);
		double observedHeading = Math.atan((observedFeatureCoords.getY() - deviceCoords.getY())
				/ (observedFeatureCoords.getX() - deviceCoords.getX()))
				- this.getHeadingRadians();
		// TODO Something wrong here (heading)

		/* Calculate the Kalman Gain */

		// Calculate innovation matrix
		Matrix hMatrix = this.createH(observedDistance, featureIndex, observedFeatureCoords, deviceCoords);
		Matrix pMatrix = this.extractSubMatrix(0, P.size() - 1, 0, P.size() - 1);

		Matrix hphMatrix = hMatrix.times(pMatrix).times(hMatrix.transpose());
		Matrix vrvMatrix = this.createVRVMatrix(observedDistance);
		Matrix innovationMatrix = hphMatrix.plus(vrvMatrix);

		Matrix kalmanGainMatrix = pMatrix.times(hMatrix.transpose()).times(innovationMatrix.inverse());

		/* Predict the distance and heading to the specified feature */
		double predictedDistanceX = featureCoords.getX() - deviceCoords.getX();
		double predictedDistanceY = featureCoords.getY() - deviceCoords.getY();
		double predictedDistance = Math.sqrt(Math.pow(predictedDistanceX, 2) + Math.pow(predictedDistanceY, 2));
		double predictedHeading = (Math.atan(predictedDistanceY / predictedDistanceX)) - this.getHeadingRadians();

		// Still need to add measurement noise to these two variables
		double[][] differenceVector = new double[2][1];
		differenceVector[0][0] = observedDistance - predictedDistance;
		differenceVector[1][0] = observedHeading - predictedHeading;
		Matrix zMinusHMatrix = new Matrix(differenceVector);

		/* Adjust state vector based on prediction */
		Matrix xMatrix = createStateVectorMatrix();
		xMatrix = xMatrix.plus(kalmanGainMatrix.times(zMinusHMatrix));

		// re-populate the state vector based on the result
		X.clear();
		double[][] x = xMatrix.getArray();
		for (int i = 0; i < x.length; i++)
			X.add(x[i][0]);

	}

	// Method for deleting a feature. Includes removing the feature from the
	// state vector and covariance matrix.
	public void deleteFeature(int featureIndex) {
		int targetIndexStart = 3 + featureIndex * 2;

		X.remove(targetIndexStart);
		X.remove(targetIndexStart);

		P.remove(targetIndexStart);
		P.remove(targetIndexStart);

		for (ArrayList<Double> row : P) {
			row.remove(targetIndexStart);
			row.remove(targetIndexStart);
		}

		numFeatures--;
	}

	// Method for adding a feature to the sate vector and covariance matrix.
	public void addFeature(double x, double y) {

		// add to state vector
		X.add(x);
		X.add(y);

		// add to covariance matrix
		// add 2 rows, then add two columns at the end

		Matrix pPhiMatrix = this.extractPPhi();

		ArrayList<Matrix> toAdd = new ArrayList<Matrix>();

		// P^phi * Jxr^T
		Matrix lowerLeftMatrix = pPhiMatrix.times(jrMatrix.transpose()).transpose();

		toAdd.add(lowerLeftMatrix);

		// numFeatures still holds the number of features not counting this new
		// feature to be added
		for (int i = 0; i < numFeatures; i++) {
			// extract the sub-matrix above the new matrix's location
			Matrix subMatrix = this.extractSubMatrix(0, 2, 3 + i * 2, 4 + i * 2);
			Matrix currMatrix = jrMatrix.times(subMatrix);
			toAdd.add(currMatrix);
		}

		// Create vrvMatrix
		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble featureCoords = new PointDouble(x, y);
		double distance = deviceCoords.computeDistanceTo(featureCoords);
		Matrix vrvMatrix = createVRVMatrix(distance);

		Matrix lowerRightMatrix = jrMatrix.times(pPhiMatrix).times(jrMatrix.transpose())
				.plus(jzMatrix.times(vrvMatrix).times(jzMatrix.transpose()));
		toAdd.add(lowerRightMatrix);

		// This part adds the last 2 rows
		for (int i = 0; i < 2; i++) {
			ArrayList<Double> currRow = new ArrayList<Double>();

			for (Matrix matrix : toAdd) {
				for (int j = 0; j < matrix.getColumnDimension(); j++)
					currRow.add(matrix.get(i, j));
			}

			P.add(currRow);
		}

		// This part adds the new 2 columns
		// Do not include the last entry in toAdd (the lower right matrix)
		// because you're not going to transpose it!
		for (int i = 0, row = 0; i < toAdd.size() - 1; i++) {
			Matrix transpose = toAdd.get(i).transpose();

			for (int j = 0; j < transpose.getRowDimension(); j++) {
				for (int k = 0; k < transpose.getColumnDimension(); k++) {
					P.get(row).add(transpose.get(j, k));
				}
				row++;
			}
		}

		numFeatures++;
	}

	/********** Methods for Creating Matrices **********/

	private Matrix createH(double observedDistance, int featureIndex, PointDouble featureCoords,
			PointDouble deviceCoords) {
		// Set-up H for the specified feature
		double[][] H = new double[2][3 + numFeatures * 2];

		double r = observedDistance; // unsure about this. might be predicted
										// distance
		double A = (featureCoords.getX() - deviceCoords.getX()) / r;
		double B = (featureCoords.getY() - deviceCoords.getY()) / r;
		double C = 0;
		double D = (featureCoords.getY() - deviceCoords.getY()) / (r * r);
		double E = (featureCoords.getX() - deviceCoords.getX()) / (r * r);
		double F = -1;

		H[0][0] = A;
		H[0][1] = B;
		H[0][2] = C;
		H[1][0] = D;
		H[1][1] = E;
		H[1][2] = F;

		int targetFeatureIndex = 3 + 2 * featureIndex;

		H[0][targetFeatureIndex] = -1 * A;
		H[0][targetFeatureIndex + 1] = -1 * B;
		H[1][targetFeatureIndex] = -1 * D;
		H[1][targetFeatureIndex + 1] = -1 * E;

		return new Matrix(H);
	}

	// Initializes the state vector
	private ArrayList<Double> createX() {
		ArrayList<Double> X = new ArrayList<Double>();
		X.add(0.0); // Device X
		X.add(0.0); // Devce Y
		X.add(0.0); // Device Theta

		Log.i("EKF", "First EVER: " + X.toString());

		return X;
	}

	// Initializes the covariance matrix
	private ArrayList<ArrayList<Double>> createP() {
		ArrayList<ArrayList<Double>> P = new ArrayList<ArrayList<Double>>();

		for (int i = 0; i < 3; i++) {
			ArrayList<Double> currRow = new ArrayList<Double>();
			for (int j = 0; j < 3; j++) {
				if (i != j)
					currRow.add(0.0);
				else
					currRow.add(P_DIAGONAL_INITIAL);
			}
			P.add(currRow);
		}

		return P;
	}

	// Returns the Jacobian matrix A based on the given deltaX and deltaY
	private Matrix createA(double deltaX, double deltaY) {
		double[][] A = { { 1, 0, -1 * deltaY }, // rightmost 0 is to be replaced
												// by - delta y
				{ 0, 1, deltaX }, // rightmost 0 is to be replaced by delta x
				{ 0, 0, 1 } };

		return new Matrix(A);
	}

	// Returns the Process Noise Q based on the given deltaX and deltaY, and
	// deltaT in radians
	private Matrix createQ(double dX, double dY, double dT) {
		double c = Q_NOISE; // will change this accdg to trial and error (accdg
							// to SLAM for dummies)
		double[][] Q = { { c * dX * dX, c * dX * dY, c * dX * dT }, { c * dY * dX, c * dY * dY, c * dY * dT },
				{ c * dT * dX, c * dT * dY, c * dT * dT } };

		return new Matrix(Q);
	}

	// The measurement noise matrix
	private Matrix createVRVMatrix(double distance) {
		Random rand = new Random();

		double[][] vrv = new double[2][2];
		// Variance of 0.01 included this way according to
		// http://www.javapractices.com/topic/TopicAction.do?Id=62
		vrv[0][0] = distance * rand.nextGaussian() * VRV_VARIANCE;
		vrv[1][1] = 1;
		return new Matrix(vrv);
	}

	// Just converts the current state vector to a Matrix object
	private Matrix createStateVectorMatrix() {
		double[][] x = new double[X.size()][1];
		for (int i = 0; i < X.size(); i++)
			x[i][0] = X.get(i);

		return new Matrix(x);
	}

	// Creates some Jacobian matrix used when adding a new feature to the
	// covariance matrix
	private Matrix createJRMatrix(double displacementX, double displacementY) {
		double[][] jr = { { 1, 0, -1 * displacementY }, { 0, 1, displacementX } };
		return new Matrix(jr);
	}

	// Creates some Jacobian matrix used when adding a new feature to the
	// covariance matrix
	private Matrix createJZMatrix(double displacementX, double displacementY, double headingRadians) {
		double[][] jz = { { Math.cos(headingRadians), -1 * displacementY }, { Math.sin(headingRadians), displacementX } };
		return new Matrix(jz);
	}
}
