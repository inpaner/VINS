package tests;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

import motionestimation.DevicePose;
import android.test.AndroidTestCase;
import ekf.EKF;

public class EKFTests extends AndroidTestCase {
    
	private EKF ekf;
	
	@Override
	protected void setUp() throws Exception {
		ekf = new EKF();
		//Pre-condition. Should assert these first or else following test is invalid
    	assertTrue(EKF.P_DIAGONAL_INITIAL == 0.1);
    	assertTrue(EKF.VRV_VARIANCE == 0.01); 
	}
	
	@Override
	protected void tearDown() throws Exception {
		ekf = null;
	}
    

	//Tests that the predict step using INS updates is correct
    public void testInsPredict() {
		
    	//INS Update 1
    	ekf.predictFromINS(Math.sqrt(2), Math.PI/4);
    	
		//Asserts that X was updated correctly
		
		DevicePose devicePose = ekf.getCurrDevicePose();
		assertEquals("X should be 1", 1.0, round2Decimals(devicePose.get_xPos()));
		assertEquals("Y should be 1", 1.0, round2Decimals(devicePose.get_yPos()));
		assertEquals("Heading should be PI/4", round2Decimals(Math.PI/4), round2Decimals(devicePose.getHeading()));
	    
		//Asserts that P was updated correctly. 	
		ArrayList<ArrayList<Double>> P = ekf.getP();
		
		assertEquals("P[0][0] should be 0.3", 0.3, roundDecimals(P.get(0).get(0), 4));
		assertEquals("P[0][1] should be 0", 0.0, roundDecimals(P.get(0).get(1), 4));
		assertEquals("P[0][2] should be -0.0215", -0.0215, roundDecimals(P.get(0).get(2), 4));
		assertEquals("P[1][0] should be 0", 0.0, roundDecimals(P.get(1).get(0), 4));
		assertEquals("P[1][1] should be 0.3", 0.3, roundDecimals(P.get(1).get(1), 4));
		assertEquals("P[1][2] should be 0.1785", 0.1785, roundDecimals(P.get(1).get(2), 4));
		assertEquals("P[2][0] should be -0.0215", -0.0215, roundDecimals(P.get(2).get(0), 4));
		assertEquals("P[2][1] should be 0.1785", 0.1785, roundDecimals(P.get(2).get(1), 4));
		assertEquals("P[2][2] should be 0.1617", 0.1617, roundDecimals(P.get(2).get(2), 4));
    } 
    
    //Tests that adding a feature is correct
    public void testAddOneFeature(){
    	
    	ekf.addFeature(0, 1);
    	
    	//Check X (state vector) contents
    	ArrayList<Double> X = ekf.getX();
    	
    	assertEquals("X's size should be 5.", 5, X.size());
    	assertEquals("X[3] should be 0", 0.0, X.get(3));
    	assertEquals("X[4] should be 1", 1.0, X.get(4));
    	
    	//Check P (covariance matrix) contents
    	
    	//Check P size
    	ArrayList<ArrayList<Double>> P = ekf.getP();
    	int rows = P.size();
    	assertEquals("P's rows should be 5", 5, rows);
    	
    	for(int i=0; i<rows; i++){
    		ArrayList<Double> row = P.get(i);
    		assertEquals("Row "+i+" should have 5 columns.", 5, row.size());
    	}
    	
    	//Check the actual contents
    	
    	//lower-left
    	assertEquals("P[3][0] should be 0.1", 0.1, roundDecimals(P.get(3).get(0), 1));
    	assertEquals("P[3][1] should be 0.0", 0.0, roundDecimals(P.get(3).get(1), 1));
    	assertEquals("P[3][2] should be 0.0", 0.0, roundDecimals(P.get(3).get(2), 1));
    	assertEquals("P[4][0] should be 0.0", 0.0, roundDecimals(P.get(4).get(0), 1));
    	assertEquals("P[4][1] should be 0.1", 0.1, roundDecimals(P.get(4).get(1), 1));
    	assertEquals("P[4][2] should be 0.0", 0.0, roundDecimals(P.get(4).get(2), 1));
    	
    	//upper-right (should be the transpose of lower-left)
    	assertEquals("P[0][3] should be 0.1", 0.1, roundDecimals(P.get(0).get(3), 1));
    	assertEquals("P[0][4] should be 0.0", 0.0, roundDecimals(P.get(0).get(4), 1));
    	assertEquals("P[1][3] should be 0.0", 0.0, roundDecimals(P.get(1).get(3), 1));
    	assertEquals("P[1][4] should be 0.1", 0.1, roundDecimals(P.get(1).get(4), 1));
    	assertEquals("P[2][3] should be 0.0", 0.0, roundDecimals(P.get(2).get(3), 1));
    	assertEquals("P[2][4] should be 0.0", 0.0, roundDecimals(P.get(2).get(4), 1));
    	
    	//lower-right
    	//cannot assert P[3][3]'s value because it is random
    	assertEquals("P[3][4] should be 0.0", 0.0, P.get(3).get(4));
    	assertEquals("P[4][3] should be 0.0", 0.0, P.get(4).get(3));
    	assertEquals("P[4][4] should be 0.1", 0.1, P.get(4).get(4));
    	
    }
    
    
    private double round2Decimals(double value){
    	return roundDecimals(value, 2);
    }
    
    private double roundDecimals(double value, int decimalPlaces){
	 BigDecimal bd = new BigDecimal(value);
	 bd = bd.setScale(decimalPlaces, RoundingMode.HALF_UP);
	 return bd.doubleValue();
    }

}
