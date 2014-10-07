package tests;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

import motionestimation.DevicePose;

import org.junit.Test;

import android.test.AndroidTestCase;
import ekf.EKF;

public class EKFTests extends AndroidTestCase {
    
    
    @Test
    public void insPredict() {
	EKF ekf = new EKF();
	ekf.predictFromINS(Math.sqrt(2), Math.PI/4);
	
	DevicePose devicePose = ekf.getCurrDevicePose();
	assertEquals("X should be 1", 1.0, round2Decimals(devicePose.get_xPos()));
	assertEquals("Y should be 1", 1.0, round2Decimals(devicePose.get_yPos()));
	assertEquals("Heading should be PI/4", round2Decimals(Math.PI/4), round2Decimals(devicePose.getHeading()));
    
	ArrayList<ArrayList<Double>> P = ekf.getP();
	
	assertEquals("Q[0][0] should be 0.1", 0.1, round2Decimals(P.get(0).get(0)));
	assertEquals("Q[0][1] should be 0.1", 0.1, round2Decimals(P.get(0).get(1)));
	assertEquals("Q[0][2] should be PI/40", round2Decimals(Math.PI/40), round2Decimals(P.get(0).get(2)));
	assertEquals("Q[1][0] should be 0.1", 0.1, round2Decimals(P.get(1).get(0)));
	assertEquals("Q[1][1] should be 0.1", 0.1, round2Decimals(P.get(1).get(1)));
	assertEquals("Q[1][2] should be PI/40", round2Decimals(Math.PI/40), round2Decimals(P.get(1).get(2)));
	assertEquals("Q[2][0] should be PI/40", round2Decimals(Math.PI/40), round2Decimals(P.get(2).get(0)));
	assertEquals("Q[2][1] should be PI/40", round2Decimals(Math.PI/40), round2Decimals(P.get(2).get(1)));
	assertEquals("Q[2][2] should be PI^2/160", round2Decimals(Math.PI*Math.PI/160), round2Decimals(P.get(2).get(2)));
    } 
    
    private double round2Decimals(double value){
	 BigDecimal bd = new BigDecimal(value);
	 bd = bd.setScale(2, RoundingMode.HALF_UP);
	 return bd.doubleValue();
    }

}
