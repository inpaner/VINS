package ekf;
import java.util.ArrayList;

import motionestimation.DevicePose;
import Jama.Matrix;
public class EKF{

    private ArrayList<Double> X; // State Vector
    private ArrayList<ArrayList<Double>> P; // Covariance Matrix
    private int numFeatures;
        
    public EKF(){
	X = createX();
	P = createP();
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
    public void insUpdate(double displacement, double heading){
	//Initialization of variables
	double headingRadians = Math.toRadians(heading);
	double displacementX = displacement * Math.cos(headingRadians);
	double displacementY = displacement * Math.sin(headingRadians);
	double displacementHeading = heading - X.get(2);
	
	//Update the state vector
	double newX = X.get(0) + displacementX;
	double newY = X.get(1) + displacementY;
	
	X.set(0, newX);
	X.set(1, newY);
	X.set(2, heading);
	
	//Update the upper left 3x3 sub-covariance matrix
	double[][] Q = createQ(displacementX, displacementY, displacementHeading);
	Matrix qMatrix = new Matrix(Q);
	
	double[][] pPhi = extractPPhi(P);
	Matrix pPhiMatrix = new Matrix(pPhi);
	
	double[][] A = createA(displacementX, displacementY); // Jacobian of Prediction Model
	Matrix aMatrix = new Matrix(A);
	
	// pPhi = A * pPphi * A^T + Q
	pPhi = aMatrix.times(pPhiMatrix).times(aMatrix.transpose()).plus(qMatrix).getArray();
	
	for(int i=0;i<pPhi.length;i++)
	    for(int j=0;j<pPhi.length;j++)
		P.get(i).set(j, pPhi[i][j]);
	
	// Update the first 3 columns of P (device to feature correlation) P_ri = A * P_ri
	
	for(int i=0;i<numFeatures;i++){
	    double[][] Pri = extractPri(P, i);
	    Matrix PriMatrix = new Matrix(Pri);
	    PriMatrix = PriMatrix.times(aMatrix);
	    Pri = PriMatrix.getArray();
	    
	    for(int j=0;j<Pri.length; j++)
	    	for(int k=0;k<Pri[j].length;k++)
	    	    P.get(j).set(k, Pri[j][k]);
	}   
    }
    
    private double[][] extractPri(ArrayList<ArrayList<Double>> P, int index){
	
	int startRowIndex = 3 + index*2;
	
	double[][] Pri = new double[2][3];
	
	for(int i=0;i<2;i++)
	    for(int j=0;j<3;j++)
		Pri[startRowIndex+i][j] = P.get(i).get(j);
		
	return Pri;
    }
    
    private double[][] extractPPhi(ArrayList<ArrayList<Double>> P ){
	double[][] pPhi = new double[3][3];
	for(int i=0;i<3;i++)
	    for(int j=0;j<3;j++)
		pPhi[i][j] = P.get(i).get(j);
	
	return pPhi;
    }
    
    
    /********** V-INS Update **********/
    
    //Method for correcting the state vector based on re-observed features.
    public void updateReobservedFeature(int featureIndex, int observedDistance, int observedHeading){
	
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
	
	double[][] vrv = new double[2][2];
	//this should be r*c, where c is a gaussian with a variance of 0.01. i don't know how that translates to code
	vrv[0][0] = r*0.01; 
	vrv[1][1] = 1; 
	Matrix vrvMatrix = new Matrix(vrv);
	
	Matrix innovationMatrix = hphMatrix.plus(vrvMatrix);
	
	Matrix kalmanGainMatrix = pMatrix.times(hMatrix.transpose()).times(innovationMatrix.inverse());
	

	
	/* Predict the distance and heading to the specified feature */
	double predictedDistanceX = featureCoords.getX() - deviceCoords.getX();
	double predictedDistanceY = featureCoords.getY() - deviceCoords.getY();
	
	//Still need to add measurement noise to these two variables
	double predictedDistance = Math.sqrt(Math.pow(predictedDistanceX,2) + Math.pow(predictedDistanceY, 2));
	double predictedHeading = Math.atan(predictedDistanceY / predictedDistanceX);
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
    
    private Matrix createStateVectorMatrix(){
	double[][] x= new double[X.size()][1];
	for(int i=0; i<X.size(); i++)
	    x[i][0] = X.get(i);
	
	return new Matrix(x);
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
	for(int i=0; i<P.size(); i++){
	    for(int j=0;j<2;j++)
		P.get(i).add(0.0); //initial covariance = 0
	}
	
	for(int i=0;i<2;i++){
	    ArrayList<Double> newRow = new ArrayList<Double>();
	    int cols = 3 + numFeatures * 2;
	    for(int j=0;j<cols;j++)
		newRow.add(0.0);
	    P.add(newRow);
	}
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
     
    //Returns the Process Noise Q based on the given deltaX and deltaY
    private double[][] createQ(double dX, double dY, double dT){
	double c = 0.1;
	double[][] Q = {{c*dX*dX, c*dX*dY, c*dX*dT},
			{c*dY*dX, c*dY*dY, c*dY*dT},
			{c*dT*dX, c*dT*dY, c*dT*dT}};
	
	return Q;
    }
    
    
}