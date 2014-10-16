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
import android.widget.TextView;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureManager;
import features.FeatureManagerListener;
import features.FeatureUpdate;

public class DriverActivity extends Activity implements SensorEventListener, FeatureManagerListener {

	private static String TAG = "Driver Activity";
	private SensorEntry nextSensorEntryToAdd;
	private EKF ekf;
	private FeatureManager featureManager;
	MotionEstimation motionEstimator;

	// TODO: maybe separate all motion estimation things to motion estimator
	private SensorManager sensorManager;
	private Sensor senAccelerometer, senGyroscope, senOrientation, senGravity, senMagnetField;

	private float mGrav[], mMag[];

	private boolean isFeaturesReady = false;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		// setContentView(R.layout.activity_driver);
		setContentView(R.layout.fastlayout);

		featureManager = new FeatureManager(this, this);
		ekf = new EKF();
		Log.i("EKF", ekf.getCurrDevicePose().toString());
		motionEstimator = new IntegrateMotionEstimation();
		nextSensorEntryToAdd = new SensorEntry();

		sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		senAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);// Sensor.TYPE_ACCELEROMETER);
		senGyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		senOrientation = sensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);

		senGravity = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
		senMagnetField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

		sensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(this, senGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(this, senOrientation, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(this, senGravity, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(this, senMagnetField, SensorManager.SENSOR_DELAY_FASTEST);

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
		if (!isFeaturesReady)
			return;

		try {
			/* TRIGGER MOTION ESTIMATION */
			DevicePose devicePose = motionEstimator.getHeadingAndDisplacement();

			StringBuilder logString = new StringBuilder();
			logString.append("Device Pose(ME): " + devicePose.toString() + "\n");

			/* PASS DISTANCE & HEADING TO EKF.insUpdate() */
			ekf.predictFromINS(devicePose.getXYDistance(), devicePose.getHeading());

			logString.append("Device Pose(After EKF Predict): " + ekf.getCurrDevicePose().toString() + "\n");

			/* TRIGGER TRIANGULATION AND GET OLD, RE-OBSERVED, AND NEW FEATURES */
			FeatureUpdate update = featureManager.getFeatureUpdate();

			logString.append("Features to Delete: " + update.getBadPointsIndex().size() + "\nFeatures to Update: "
					+ update.getCurrentPoints().size() + "\nFeatures to Add: " + update.getNewPoints().size() + "\n");

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

			devicePose = ekf.getCurrDevicePose();
			logString.append("Device Pose(EKF): " + devicePose.toString() + "\n");
			TextView tv = (TextView) findViewById(R.id.debugTextView);
			tv.setText(devicePose.toString() + "\n" + tv.getText());

			Log.i(TAG, logString.toString());

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
		int i = 0;
		while (nextSensorEntryToAdd.status != SensorEntry.FULL)
			i++;
		motionEstimator.inputData(nextSensorEntryToAdd);
		nextSensorEntryToAdd = new SensorEntry();
	}

	@Override
	public void onSensorChanged(SensorEvent sensorEvent) {

		nextSensorEntryToAdd.status = SensorEntry.WRITING;
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
			//
			// nextSensorEntryToAdd.setOrient_x(x);
			// nextSensorEntryToAdd.setOrient_y(y);
			// nextSensorEntryToAdd.setOrient_z(z);
		} else if (mySensor.getType() == Sensor.TYPE_GRAVITY) {
			mGrav = lowpass(sensorEvent.values.clone(), mGrav);
		} else if (mySensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
			mMag = lowpass(sensorEvent.values.clone(), mMag);
		}

		if (mGrav != null && mMag != null) {

			float[] rotMat = new float[9];
			if (SensorManager.getRotationMatrix(rotMat, null, mGrav, mMag)) {
				float[] remap = new float[9];

				SensorManager.remapCoordinateSystem(rotMat, SensorManager.AXIS_X, SensorManager.AXIS_Z, remap);

				float[] orient = new float[3];

				SensorManager.getOrientation(rotMat, orient);

				float heading = 0;
				if (heading < 0) {
					heading += 360;
				}

				for (int i = 0; i < orient.length; ++i) {
					orient[i] = (float) Math.toDegrees(orient[i]);
					if (orient[i] < 0)
						orient[i] += 360;
				}

				nextSensorEntryToAdd.setOrient_x(orient[0]); // rotation by z
				nextSensorEntryToAdd.setOrient_y(orient[1]);
				nextSensorEntryToAdd.setOrient_z(orient[2]);

				// Log.i(TAG, orient[0] + "," + orient[1] + "," + orient[2]);
			}
		}

		nextSensorEntryToAdd.status = SensorEntry.FULL;
	}

	private float[] lowpass(float[] curr, float[] prev) {
		// if(prev != null)
		// for(int i=0; i < curr.length; ++i)
		// curr[i] += ALPHA * (curr[i]-prev[i]);

		return curr;
	}

	@Override
	public void initDone() {
		isFeaturesReady = true;
	}
}
