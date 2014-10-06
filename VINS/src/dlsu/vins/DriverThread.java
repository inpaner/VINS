package dlsu.vins;

import java.util.Calendar;

public class DriverThread extends Thread{
    
    private Calendar lastUpdateTime;
    private long millisInterval = 333;
    
    public DriverThread(){
	lastUpdateTime = Calendar.getInstance();
    }
    
    public void run(){
	
	while(true){
	    
	    Calendar currUpdateTime = Calendar.getInstance();
	    
	    if(currUpdateTime.getTimeInMillis() - lastUpdateTime.getTimeInMillis() >= millisInterval ){
		/* TRIGGER MOTION ESTIMATION */
		/* PASS DISTANCE & HEADING TO EKF.insUpdate()*/
		/* TRIGGER TRIANGULATION AND GET OLD, RE-OBSERVED, AND NEW FEATURES*/

		/* LOOP THROUGH THE RETURNED FEATURES*/
        		/* IF OLD FEATURE TYPE, CALL EKF.removeFeature(featureIndex)*/
        		/* IF RE-OBSERVED FEATURE, CALL EKF.updateReobservedFeature(featureIndex, observedDistance, observedHeading)*/
        		/* IF NEW FEATURE, CALL EKF.addFeature(x, y)*/
	    }
	    
	}
    }

}
