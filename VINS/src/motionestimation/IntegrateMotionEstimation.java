package motionestimation;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.concurrent.Semaphore;

import Jama.Matrix;
import android.util.Log;

public class IntegrateMotionEstimation implements MotionEstimation {
	// timer counters
	long start = -1;
	long curr = -1;

	// sensor data
	ArrayList<SensorEntry> entries = new ArrayList<SensorEntry>();
	
	// overflow data, add when next time
	ArrayList<SensorEntry> overflow = new ArrayList<SensorEntry>();

	
	// TODO: temporary concurrency fix
	Semaphore mutex = new Semaphore(1);

	public void inputData(SensorEntry s) {
		if (start == -1)
			startTimer();
		else
			curr = Calendar.getInstance().getTimeInMillis();

		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		if (start + 333 >= curr) {
			entries.add(s);
		} else {
			overflow.add(s);
		}
		
		mutex.release();
	}

	public DevicePose getHeadingAndDisplacement() throws Exception {
		
		double heading;
		double orientXAvg, orientYAvg, orientZAvg;

		orientXAvg = 0;
		orientYAvg = 0;
		orientZAvg = 0;
		
		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		for (SensorEntry s : entries){
			orientXAvg += s.getOrient_x();
			orientYAvg += s.getOrient_y();
			orientZAvg += s.getOrient_z();
		}
		
		if (entries.size() != 0) {
			orientXAvg /= entries.size();
			orientYAvg /= entries.size();
			orientZAvg /= entries.size();
		}
		
		mutex.release();

		heading = orientXAvg;
		orientXAvg = Math.toRadians(orientXAvg);
		orientYAvg = Math.toRadians(orientYAvg);
		orientZAvg = Math.toRadians(orientZAvg);

		double cosVal, sinVal;

		// TODO: might need to double check this
		// orientX is actually rotation along Z, hence rotZArr
		// orientY is the rotation along X, hence rotYArr
		// orientZ is rotation along Y, hence rotZArr
		
		cosVal = Math.cos(orientXAvg);
		sinVal = Math.sin(orientXAvg);
		double[][] rotZArr = { { cosVal, -sinVal, 0 }, { sinVal, cosVal, 0 }, { 0, 0, 1 } };
		Matrix rotZ = new Matrix(rotZArr);

		cosVal = Math.cos(orientYAvg);
		sinVal = Math.sin(orientYAvg);
		double[][] rotXArr = { { 1, 0, 0 }, { 0, cosVal, -sinVal }, { 0, sinVal, cosVal } };
		Matrix rotX = new Matrix(rotXArr);
		
		cosVal = Math.cos(orientZAvg);
		sinVal = Math.sin(orientZAvg);
		double[][] rotYArr = { { cosVal, 0, sinVal }, { 0, 1, 0 }, { -sinVal, 0, cosVal } };
		Matrix rotY = new Matrix(rotYArr);

		Matrix rotFinal = rotX.times(rotY).times(rotZ).transpose();

		// D O U B L E I N T E G R A T I O N
		double[] pos = new double[3];

		// x = a*t^2/2 (t is in seconds)

		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		for (SensorEntry s : entries){
			pos[0] += s.getAcc_x();
			pos[1] += s.getAcc_y();
			pos[2] += s.getAcc_z();
		}
		
		if (entries.size() > 0){
			pos[0] /= entries.size();
			pos[1] /= entries.size();
			pos[2] /= entries.size();
		}
		
		mutex.release();
		
		Matrix xyzMatrix = new Matrix(new double[][] { pos });

		xyzMatrix = xyzMatrix.times(rotFinal);

		pos[0] = xyzMatrix.get(0, 0);
		pos[1] = xyzMatrix.get(0, 1);
		pos[2] = xyzMatrix.get(0, 2);

		pos[0] *= Math.pow((curr - start), 2) / 2000000;
		pos[1] *= Math.pow((curr - start), 2) / 2000000;
		pos[2] *= Math.pow((curr - start), 2) / 2000000;

//		Log.i("ME", heading + " " + numInstances + "\n");

		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// clear all the arraylist
		entries.clear();
		
		// if there are overflows
		for (int i = 0; i < overflow.size(); ++i) {
			entries.add(overflow.get(i));
		}

		overflow.clear();
		
		mutex.release();

		// and reset the timers
		start = -1;
		curr = -1;

//		Log.i("ME", "POS Vector: " + pos[0] + " " + pos[1] + " " + pos[2]);

		DevicePose devicePose = new DevicePose(pos[0], pos[1], pos[2], heading);
		devicePose.setRotWorld(rotFinal);
		
		return devicePose;
	}

	public void startTimer() {
		start = Calendar.getInstance().getTimeInMillis();
		curr = Calendar.getInstance().getTimeInMillis();
	}
}

// for mico
// public DevicePose getHeadingAndDisplacement() throws Exception {
// try {
// mutex.acquire();
// } catch (InterruptedException e) {
// // TODO Auto-generated catch block
// e.printStackTrace();
// }
//
// // building world rotation matrix
//
// Mat rotX, rotY, rotZ, temp, rotWorld;
//
// double orientXAvg, orientYAvg, orientZAvg;
//
// orientXAvg = 0;
// for (Double d : orientx)
// orientXAvg += d;
//
// if (orientx.size() != 0)
// orientXAvg /= orientx.size();
//
// orientYAvg = 0;
// for (Double d : orienty)
// orientYAvg += d;
//
// if (orienty.size() != 0)
// orientYAvg /= orienty.size();
//
// orientZAvg = 0;
// for (Double d : orientz)
// orientZAvg += d;
//
// if (orientz.size() != 0)
// orientZAvg /= orientz.size();
//
// rotX = Mat.zeros(3, 3, CvType.CV_64F);
// rotY = Mat.zeros(3, 3, CvType.CV_64F);
// rotZ = Mat.zeros(3, 3, CvType.CV_64F);
// temp = Mat.zeros(3, 3, CvType.CV_64F);
// rotWorld = Mat.zeros(0, 0, CvType.CV_64F);
//
// rotX.put(2, 2, 1);
// rotY.put(1, 1, 1);
// rotZ.put(0, 0, 1);
//
// double cosVal, sinVal;
//
// cosVal = Math.cos(orientXAvg);
// sinVal = Math.sin(orientXAvg);
//
// rotX.put(0, 0, cosVal, -sinVal);
// rotX.put(1, 0, sinVal, cosVal);
//
// cosVal = Math.cos(orientYAvg);
// sinVal = Math.sin(orientYAvg);
//
// rotY.put(0, 0, cosVal, sinVal);
// rotY.put(2, 0, -sinVal, cosVal);
//
// cosVal = Math.cos(orientZAvg);
// sinVal = Math.sin(orientZAvg);
//
// rotZ.put(1, 1, cosVal, -sinVal);
// rotZ.put(2, 1, sinVal, cosVal);
//
// Core.gemm(rotX, rotY, 1, Mat.zeros(0, 0, CvType.CV_64F), 0, temp);
// Core.gemm(temp, rotZ, 1, Mat.zeros(0, 0, CvType.CV_64F), 0, rotWorld);
// rotWorld = rotWorld.t();
//
// Mat vec, rev;
// vec = Mat.zeros(1, 3, CvType.CV_64F);
// rev = Mat.zeros(1, 3, CvType.CV_64F);
//
// // D O U B L E I N T E G R A T I O N
// double[] pos = new double[3];
//
// // x = a*t^2/2 (t is in seconds)
//
// for (Double d : accx)
// pos[0] += d;
// pos[0] /= accx.size();
//
// for (Double d : accy)
// pos[1] += d;
// pos[1] /= accy.size();
//
// for (Double d : accz)
// pos[2] += d;
// pos[2] /= accz.size();
//
// vec.put(0, 0, pos[0]);
// vec.put(0, 1, pos[1]);
// vec.put(0, 2, pos[2]);
//
// Core.gemm(vec, rotWorld, 1, Mat.zeros(0, 0, CvType.CV_64F), 0, rev);
//
// pos[0] = rev.get(0, 0)[0];
// pos[1] = rev.get(0, 1)[0];
// pos[2] = rev.get(0, 2)[0];
//
// pos[0] *= Math.pow((curr - start), 2) / 2000000;
// pos[1] *= Math.pow((curr - start), 2) / 2000000;
// pos[2] *= Math.pow((curr - start), 2) / 2000000;
//
// // average all the gyroscope readings
// int numInstances = orientx.size();
// double heading = 0;
// for (Double d : orientx)
// heading += d;
//
// if (numInstances != 0)
// heading /= numInstances;
//
// // clear all the arraylists
// accx.clear();
// accy.clear();
// accz.clear();
// orientx.clear();
// orienty.clear();
// orientz.clear();
//
// // if there are overflows
// for (int i = 0; i < overflow.size(); i += 6) {
// accx.add(overflow.get(i));
// accy.add(overflow.get(i + 1));
// accz.add(overflow.get(i + 2));
// orientx.add(overflow.get(i + 3));
// orienty.add(overflow.get(i + 4));
// orientz.add(overflow.get(i + 5));
// }
//
// overflow.clear();
//
// // and reset the timers
// start = -1;
// curr = -1;
//
// mutex.release();
// return new DevicePose(pos[0], pos[1], pos[2], heading);
// }
