package dlsu.vins;

import java.util.Timer;
import java.util.TimerTask;

import motionestimation.DevicePose;
import motionestimation.IntegrateMotionEstimation;
import motionestimation.MotionEstimation;
import motionestimation.SensorEntry;
import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.os.Bundle;
import ekf.EKF;

public class DriverActivity extends Activity implements SensorEventListener{

    private SensorEntry nextSensorEntryToAdd;
    private EKF ekf;
    MotionEstimation motionEstimator;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
	super.onCreate(savedInstanceState);
	setContentView(R.layout.activity_driver);
	
	ekf = new EKF();
	motionEstimator = new IntegrateMotionEstimation();
	nextSensorEntryToAdd = new SensorEntry();
	startInertialSensorLogging();
	startDriver();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
	// TODO Auto-generated method stub
	
    }
    
    private void startDriver(){
	Timer recordTimer = new Timer();
	recordTimer.scheduleAtFixedRate(new TimerTask() {          
            @Override
            public void run() {
        	runOneCycle();
            }
	}, 0, 333);
    }
    
    private void runOneCycle(){
	/* TRIGGER MOTION ESTIMATION */
	DevicePose devicePose = motionEstimator.getHeadingAndDisplacement();
	
	/* PASS DISTANCE & HEADING TO EKF.insUpdate()*/
	ekf.predictFromINS(devicePose.getXYDistance(), devicePose.getHeading());
	
	
	/* TRIGGER TRIANGULATION AND GET OLD, RE-OBSERVED, AND NEW FEATURES*/

	/* LOOP THROUGH THE RETURNED FEATURES*/
		/* IF OLD FEATURE TYPE, CALL EKF.removeFeature(featureIndex)*/
		/* IF RE-OBSERVED FEATURE, CALL EKF.updateReobservedFeature(featureIndex, observedDistance, observedHeading)*/
		/* IF NEW FEATURE, CALL EKF.addFeature(x, y)*/
    }
    
    private void startInertialSensorLogging(){
	Timer recordTimer = new Timer();
	recordTimer.scheduleAtFixedRate(new TimerTask() {          
            @Override
            public void run() {
        	recordSensorEntry();
            }
	}, 0, 10);
    }
    
    private void recordSensorEntry(){
	motionEstimator.inputData(nextSensorEntryToAdd);
	nextSensorEntryToAdd = new SensorEntry();
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {

	Sensor mySensor = sensorEvent.sensor;
	 
	    if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
	    	
	    	//add to the sensor entry batch if time to add

		float x = sensorEvent.values[0];
	        float y = sensorEvent.values[1];
	        float z = sensorEvent.values[2];
	    	
	    	nextSensorEntryToAdd.setAcc_x(x);
	    	nextSensorEntryToAdd.setAcc_y(y);
	    	nextSensorEntryToAdd.setAcc_z(z);
	    
	    }
	    else if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {
	    	
	    	//add to the sensor entry batch if time to add

		float x = sensorEvent.values[0];
	        float y = sensorEvent.values[1];
	        float z = sensorEvent.values[2];
	    	
	    	nextSensorEntryToAdd.setGyro_x(x);
	    	nextSensorEntryToAdd.setGyro_y(y);
	    	nextSensorEntryToAdd.setGyro_z(z);
	    }
	    else if (mySensor.getType() == Sensor.TYPE_ORIENTATION) {
	    	
	    	//add to the sensor entry batch if time to add

		float x = sensorEvent.values[0];
	        float y = sensorEvent.values[1];
	        float z = sensorEvent.values[2];
	    	
	    	nextSensorEntryToAdd.setOrient_x(x);
	    	nextSensorEntryToAdd.setOrient_y(y);
	    	nextSensorEntryToAdd.setOrient_z(z);
	    }
	    
	    
    }
    
}
