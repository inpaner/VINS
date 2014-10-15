package dlsu.vins;

import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;

import motionestimation.DevicePose;
import motionestimation.IntegrateMotionEstimation;
import motionestimation.MotionEstimation;
import motionestimation.SensorEntry;
import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureManager;
import features.FeatureUpdate;

public class DriverActivity extends Activity implements SensorEventListener {

	private static String TAG = "Driver Activity";
	private SensorEntry nextSensorEntryToAdd;
	private EKF ekf;
	private FeatureManager featureManager;
	MotionEstimation motionEstimator;

	// TODO: maybe separate all motion estimation things to motion estimator
	private SensorManager sensorManager;
	private Sensor senAccelerometer, senGyroscope, senOrientation;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		//setContentView(R.layout.activity_driver);
		setContentView(R.layout.fastlayout);
		
		featureManager = new FeatureManager(this);
		ekf = new EKF();
		motionEstimator = new IntegrateMotionEstimation();
		nextSensorEntryToAdd = new SensorEntry();
		
		sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		senAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);//Sensor.TYPE_ACCELEROMETER);
		senGyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		senOrientation = sensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
		sensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, senGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, senOrientation, SensorManager.SENSOR_DELAY_FASTEST);

		startInertialSensorLogging();
		startDriver();
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub

	}

	private void startDriver() {
		Timer recordTimer = new Timer();
		recordTimer.scheduleAtFixedRate(new TimerTask() {
			@Override
			public void run() {
				runOneCycle();
			}
		}, 0, 3000);
	}

	private void runOneCycle() {
		try { // TODO: temporary fix for opencv not loading is exception handling
			
			/* TRIGGER MOTION ESTIMATION */
			DevicePose devicePose = motionEstimator.getHeadingAndDisplacement();
		
			Log.i(TAG, devicePose.get_xPos() + " " + devicePose.get_yPos() + " " + devicePose.get_zPos() + " " + devicePose.getHeading());

			/* PASS DISTANCE & HEADING TO EKF.insUpdate() */
			ekf.predictFromINS(devicePose.getXYDistance(), devicePose.getHeading());

			// Log.i(TAG, devicePose.get_xPos() + " " + devicePose.get_yPos() + " " + devicePose.get_zPos() + " " + devicePose.getHeading());
			
			/* TRIGGER TRIANGULATION AND GET OLD, RE-OBSERVED, AND NEW FEATURES */
			FeatureUpdate update = featureManager.getFeatureUpdate();

			Log.i(TAG,	"Features to Delete: " + update.getBadPointsIndex().size() +
							"\nFeatures to Update: " +	update.getCurrentPoints().size() + 
							"\nFeatures to Add: " + update.getNewPoints().size());
			
			/* LOOP THROUGH THE RETURNED FEATURES */
			
			/* IF OLD FEATURE TYPE, CALL EKF.removeFeature(featureIndex) */
			Collections.reverse(update.getBadPointsIndex());
			for (Integer index : update.getBadPointsIndex())
				ekf.deleteFeature(index);

			/*
			 * IF RE-OBSERVED FEATURE, CALL
			 * EKF.updateReobservedFeature(featureIndex, observedDistance,
			 * observedHeading)
			 */
			int i = 0;
			for (PointDouble featpos : update.getCurrentPoints())
				ekf.updateFromReobservedFeature(i++, featpos.getX(), featpos.getY());

			/* IF NEW FEATURE, CALL EKF.addFeature(x, y) */
			for (PointDouble featpos : update.getNewPoints())
				ekf.addFeature(featpos.getX(), featpos.getY());

			//devicePose = ekf.getCurrDevicePose();
			//Log.i("Driver", devicePose.get_xPos() + "\n" + devicePose.get_yPos() + "\n" + devicePose.get_zPos() + "\n" + devicePose.getHeading());
		
		} catch (Exception e) { 
			// if anything goes wrong we cry
			
			Log.e(TAG, e.toString(), e);
		}
	}

	private void startInertialSensorLogging() {
		Timer recordTimer = new Timer();
		recordTimer.scheduleAtFixedRate(new TimerTask() {
			@Override
			public void run() {
				recordSensorEntry();
			}
		}, 0, 10);
	}

	private void recordSensorEntry() {
		motionEstimator.inputData(nextSensorEntryToAdd);
		nextSensorEntryToAdd = new SensorEntry();
	}

	@Override
	public void onSensorChanged(SensorEvent sensorEvent) {

		Sensor mySensor = sensorEvent.sensor;

		if (mySensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {

			// add to the sensor entry batch if time to add

			float x = sensorEvent.values[0];
			float y = sensorEvent.values[1];
			float z = sensorEvent.values[2];

			nextSensorEntryToAdd.setAcc_x(x);
			nextSensorEntryToAdd.setAcc_y(y);
			nextSensorEntryToAdd.setAcc_z(z);

		} else if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {

			// add to the sensor entry batch if time to add

			float x = sensorEvent.values[0];
			float y = sensorEvent.values[1];
			float z = sensorEvent.values[2];

			nextSensorEntryToAdd.setGyro_x(x);
			nextSensorEntryToAdd.setGyro_y(y);
			nextSensorEntryToAdd.setGyro_z(z);
		} else if (mySensor.getType() == Sensor.TYPE_ORIENTATION) {

			// add to the sensor entry batch if time to add

			float x = sensorEvent.values[0];
			float y = sensorEvent.values[1];
			float z = sensorEvent.values[2];

			nextSensorEntryToAdd.setOrient_x(x);
			nextSensorEntryToAdd.setOrient_y(y);
			nextSensorEntryToAdd.setOrient_z(z);
		}

	}

}
