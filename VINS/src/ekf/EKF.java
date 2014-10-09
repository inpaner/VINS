package ekf;
import java.util.ArrayList;
import java.util.Random;

import motionestimation.DevicePose;
import Jama.Matrix;
public class EKF{

    private ArrayList<Double> X; // State Vector
    private ArrayList<ArrayList<Double>> P; // Covariance Matrix
    private int numFeatures;
        
    private Matrix jrMatrix;
    private Matrix jzMatrix;
    private Matrix vrvMatrix;
    
    
    public EKF(){
	X = createX();
	P = createP();
	
	jrMatrix = this.createJRMatrix(0, 0);
	jzMatrix = this.createJZMatrix(0, 0, 0);
    }
    
    public ArrayList<ArrayList<Double>> getP(){
	return P;
    }
    
    public DevicePose getCurrDevicePose(){
	PointDouble deviceCoords = getDeviceCoords();
	DevicePose pose = new DevicePose(deviceCoords.getX(), deviceCoords.getY(), 0, getHeading());
	return pose;
    }
    
    private double getHeading(){
	return X.get(2);
    }
    
    /********** INS Update **********/
    
    //Performs the state update depending on displacement in meters, and heading in degrees
    public void predictFromINS(double displacement, double headingRadians){
	//Initialization of variables
	double displacementX = displacement * Math.cos(headingRadians);
	double displacementY = displacement * Math.sin(headingRadians);
	double displacementHeading = headingRadians - X.get(2);
	
	//Update the state vector
	double newX = X.get(0) + displacementX;
	double newY = X.get(1) + displacementY;
	
	X.set(0, newX);
	X.set(1, newY);
	X.set(2, headingRadians);
	
	//Update the upper left 3x3 sub-covariance matrix
	double[][] Q = createQ(displacementX, displacementY, displacementHeading);
	Matrix qMatrix = new Matrix(Q);
	
	Matrix pPhiMatrix = this.extractPPhi();
	
	double[][] A = createA(displacementX, displacementY); // Jacobian of Prediction Model
	Matrix aMatrix = new Matrix(A);
	
	// pPhi = A * pPphi * A^T + Q
	pPhiMatrix = aMatrix.times(pPhiMatrix).times(aMatrix.transpose()).plus(qMatrix);
	
	for(int i=0;i<pPhiMatrix.getRowDimension();i++)
	    for(int j=0;j<pPhiMatrix.getColumnDimension();j++)
		P.get(i).set(j, pPhiMatrix.get(i, j));
	
	// Update the first 3 columns of P (device to feature correlation) P_ri = A * P_ri
	
	for(int i=0;i<numFeatures;i++){
	    Matrix PriMatrix = extractPri(i);
	    PriMatrix = PriMatrix.times(aMatrix);
	    
	    for(int j=0;j<PriMatrix.getRowDimension(); j++)
	    	for(int k=0;k<PriMatrix.getColumnDimension();k++)
	    	    P.get(j).set(k, PriMatrix.get(j, k));
	}   
	
	//Update Jr and Jz matrices
	jrMatrix = this.createJRMatrix(displacementX, displacementY);
	jzMatrix = this.createJZMatrix(displacementX, displacementY, headingRadians);
    }
    
    private Matrix extractPri(int index){
	int startRowIndex = 3 + index*2;
	
	return this.extractSubMatrix(startRowIndex, startRowIndex+1, 0, 2);
    }
    
    private Matrix extractPPhi(){
	return this.extractSubMatrix(0, 2, 0, 2);
    }
    
    
    /********** V-INS Update **********/
    
    //Method for correcting the state vector based on re-observed features.
    public void updateFromReobservedFeature(int featureIndex, int observedDistance, int observedHeading){
	
	PointDouble featureCoords = this.getFeatureCoordsFromStateVector(featureIndex);
	PointDouble deviceCoords = this.getDeviceCoords();
	
	/* Calculate the Kalman Gain */
	
	//Set-up H for the specified feature
	double[][] H = new double[2][3+numFeatures*2];
	
	double r = observedDistance; //unsure about this. might be predicted distance
	double A = (featureCoords.getX() - deviceCoords.getX())/r;
	double B = (featureCoords.getY() - deviceCoords.getY())/r;
	double C = 0;
	double D = (featureCoords.getY() - deviceCoords.getY())/(r*r);
	double E = (featureCoords.getX() - deviceCoords.getX())/(r*r);
	double F = -1;
	
	
	H[0][0] = A;
	H[0][1] = B;
	H[0][2] = C;
	H[1][0] = D;
	H[1][1] = E;
	H[1][2] = F;
	
	int targetFeatureIndex = 3 + 2*featureIndex;
	
	H[0][targetFeatureIndex] = -1*A;
	H[0][targetFeatureIndex+1] = -1*B;
	H[1][targetFeatureIndex] = -1*D;
	H[1][targetFeatureIndex+1] = -1*E;
	
	
	//Calculate innovation matrix
	Matrix hMatrix = new Matrix(H);
	Matrix pMatrix = new Matrix((double[][])P.toArray());
	Matrix hphMatrix = hMatrix.times(pMatrix).times(hMatrix.transpose());
	
	vrvMatrix = this.createVRVMatrix(r);
	Matrix innovationMatrix = hphMatrix.plus(vrvMatrix);
	
	Matrix kalmanGainMatrix = pMatrix.times(hMatrix.transpose()).times(innovationMatrix.inverse());
	

	
	/* Predict the distance and heading to the specified feature */
	double predictedDistanceX = featureCoords.getX() - deviceCoords.getX();
	double predictedDistanceY = featureCoords.getY() - deviceCoords.getY();
	double predictedDistance = Math.sqrt(Math.pow(predictedDistanceX,2) + Math.pow(predictedDistanceY, 2));
	double predictedHeading = (Math.atan(predictedDistanceY / predictedDistanceX)) - this.getHeading();
	
	//Still need to add measurement noise to these two variables
	double[][] differenceVector = new double[2][1];
	differenceVector[0][0] = observedDistance - predictedDistance;
	differenceVector[1][0] = observedHeading - predictedHeading;
	Matrix zMinusHMatrix = new Matrix(differenceVector);
	
	
	/* Adjust state vector based on prediction */
	Matrix xMatrix = createStateVectorMatrix();
	xMatrix = xMatrix.plus(kalmanGainMatrix.times(zMinusHMatrix));
	
	//re-populate the state vector based on the result
	X.clear();
	double[][] x = xMatrix.getArray();
	for(int i=0; i<x.length;i++)
	    X.add(x[i][0]);
	    
    }
    
    //Method for deleting a feature. Includes removing the feature from the state vector and covariance matrix.
    public void deleteFeature(int featureIndex){
	int targetIndexStart = 3 + featureIndex * 2;
	
	P.remove(targetIndexStart);
	P.remove(targetIndexStart);
	
	for(ArrayList<Double> row: P){
	    row.remove(targetIndexStart);
	    row.remove(targetIndexStart);
	}
    }
    
    //Method for adding a feature to the sate vector and covariance matrix.
    public void addFeature(double x, double y){
	
	numFeatures++;
	
	//add to state vector
	X.add(x);
	X.add(y);
	
	//add to covariance matrix
	//add 2 rows, then add two columns at the end

	Matrix pPhiMatrix = this.extractPPhi();
	
	ArrayList<Matrix> toAdd = new ArrayList<Matrix>();
	
	//P^phi * Jxr^T
	Matrix lowerLeftMatrix = pPhiMatrix.times(jrMatrix.transpose()); // this is lower left. add it to P later on	
	
	toAdd.add(lowerLeftMatrix);
	
	for(int i=0;i<numFeatures;i++){
	    Matrix subMatrix = this.extractSubMatrix(3 + i*2, 4 + i*2, 0, 2);
	    Matrix currMatrix = jrMatrix.times(subMatrix);
	    toAdd.add(currMatrix);
	}
	
	//This part adds the new 2 columns
	for(int i=0, row = 0; i<toAdd.size(); i++){
	    Matrix transpose = toAdd.get(i).transpose();
	    
	    for(int j=0;j<transpose.getRowDimension();j++){
		for(int k=0;k<transpose.getColumnDimension();k++){
		    P.get(row).add(transpose.get(j,k));
		}
		row++;
	    }
	}
	
	Matrix lowerRightMatrix = jrMatrix.times(pPhiMatrix).times(jrMatrix.transpose()).plus(jzMatrix.times(vrvMatrix).times(jzMatrix.transpose()));
	toAdd.add(lowerRightMatrix);

	//This part adds the last 2 rows
	for(int i=0;i<2;i++){
	    ArrayList<Double> currRow = new ArrayList<Double>();
	    
	    for(Matrix matrix: toAdd){
		for(int j=0; j<matrix.getColumnDimension();j++)
		    currRow.add(matrix.get(i, j));
	    }
	    
	    P.add(currRow);
	}
    }
    
    private PointDouble getDeviceCoords(){
	PointDouble point = new PointDouble(X.get(0), X.get(1)); 
	return point;
    }
    
    private PointDouble getFeatureCoordsFromStateVector(int featureIndex){

	int stateVectorIndexOfFeature = 3 + featureIndex * 2;
	double targetFeatureX = X.get(stateVectorIndexOfFeature);
	double targetFeatureY = X.get(stateVectorIndexOfFeature+1);
	
	PointDouble point = new PointDouble(targetFeatureX, targetFeatureY); 
	
	return point;
    } 
    
    private Matrix extractSubMatrix(int startRow, int endRow, int startCol, int endCol){
	double[][] sub = new double[endRow-startRow+1][endCol-startCol+1];
	for(int i=startRow; i<=endRow; i++)
	    for(int j=startCol; j<=endCol; j++)
		sub[i-startRow][j-startCol] = P.get(i).get(j);
	 return new Matrix(sub);
    }
    
    private Matrix createVRVMatrix(double distance){
	Random rand = new Random();
	
	double[][] vrv = new double[2][2];
	//Variance of 0.01 included this way according to http://www.javapractices.com/topic/TopicAction.do?Id=62
	vrv[0][0] = distance*rand.nextGaussian()*0.01; 
	vrv[1][1] = 1; 
	return new Matrix(vrv);	
    }
    
    private Matrix createStateVectorMatrix(){
	double[][] x= new double[X.size()][1];
	for(int i=0; i<X.size(); i++)
	    x[i][0] = X.get(i);
	
	return new Matrix(x);
    }
    
    private Matrix createJRMatrix(double displacementX, double displacementY){
	double[][] jr = {{1, 0, -1*displacementY}, {0, 1, displacementX}};
	return new Matrix(jr);
    }
    
    private Matrix createJZMatrix(double displacementX, double displacementY, double headingRadians){
	double[][] jz = {{Math.cos(headingRadians), -1*displacementY}, {Math.sin(headingRadians), displacementX}};
	return new Matrix(jz);
    }
    
    /********** Initializations **********/
  
    //Initializes the state vector
    private ArrayList<Double> createX(){
	ArrayList<Double> X = new ArrayList<Double>();
	X.add(0.0); // Device X
	X.add(0.0); // Devce Y
	X.add(0.0); // Device Theta
	return X;
    }
    
    //Initializes the covariance matrix
    private ArrayList<ArrayList<Double>> createP(){
	ArrayList<ArrayList<Double>> P = new ArrayList<ArrayList<Double>>();
	
	for(int i=0;i<3;i++){
	    ArrayList<Double> currRow = new ArrayList<Double>();
	    for(int j=0;j<3;j++){
		currRow.add(0.0);
	    }
	    P.add(currRow);
	}
	
	return P;
    }
    
    //Returns the Jacobian matrix A based on the given deltaX and deltaY
    private double[][] createA(double deltaX, double deltaY){
	double[][] A = {{1, 0, -1*deltaY}, //rightmost 0 is to be replaced by - delta y
			{0, 1, deltaX}, //rightmost 0 is to be replaced by delta x
			{0, 0, 1}};
	
	return A;
    }
     
    //Returns the Process Noise Q based on the given deltaX and deltaY, and deltaT in radians
    private double[][] createQ(double dX, double dY, double dT){
	double c = 0.1; // will change this accdg to trial and error (accdg to SLAM for dummies)
	double[][] Q = {{c*dX*dX, c*dX*dY, c*dX*dT},
			{c*dY*dX, c*dY*dY, c*dY*dT},
			{c*dT*dX, c*dT*dY, c*dT*dT}};
	
	return Q;
    }
    
}