package motionestimation;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.concurrent.Semaphore;

public class IntegrateMotionEstimation implements MotionEstimation {
	// timer counters
	long start = -1;
	long curr = -1;
	
	// accelerometer data
	ArrayList<Double> accx = new ArrayList<Double>();
	ArrayList<Double> accy = new ArrayList<Double>();
	ArrayList<Double> accz = new ArrayList<Double>();
	
	// gyroscope data
	ArrayList<Double> orientx = new ArrayList<Double>();
	ArrayList<Double> orienty = new ArrayList<Double>();
	ArrayList<Double> orientz = new ArrayList<Double>();
	
	// overflow data, add when next time
	ArrayList<Double> overflow = new ArrayList<Double>();
	
	// TODO: temporary concurrency fix
	Semaphore mutex = new Semaphore(1);
	
	// format is [accx y z gyrox y z]
	public void inputData(SensorEntry s) {
		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (start == -1)
			startTimer();
		else
			curr = Calendar.getInstance().getTimeInMillis();
		
		if (start + 333 >= curr) {
			accx.add(s.getAcc_x());
			accy.add(s.getAcc_y());
			accz.add(s.getAcc_z());
			
			orientx.add(s.getOrient_x());
			orienty.add(s.getOrient_y());
			orientz.add(s.getOrient_z());
		}
		else {
			overflow.add(s.getAcc_x());
			overflow.add(s.getAcc_y());
			overflow.add(s.getAcc_z());
			overflow.add(s.getOrient_x());
			overflow.add(s.getOrient_y());
			overflow.add(s.getOrient_z());
		}
		mutex.release();
	}
	
	public DevicePose getHeadingAndDisplacement() {
		try {
			mutex.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		// D O U B L E I N T E G R A T I O N
		double[] pos = new double[3];
		
		// x = a*t^2/2 (t is in seconds)
		for (Double d : accx)
			pos[0] += d;
		pos[0] /= accx.size();
		
		for (Double d : accy)
			pos[1] += d;
		pos[1] /= accy.size();
		
		for (Double d : accz)
			pos[2] += d;
		pos[2] /= accz.size();
		
		pos[0] *= Math.pow((curr-start),2)/2000;
		pos[1] *= Math.pow((curr-start),2)/2000;
		pos[2] *= Math.pow((curr-start),2)/2000;
		
		// average all the gyroscope readings
		int numInstances = orientx.size();
		double heading = 0;
		for (Double d : orientx)
			heading += d;
		
		if(numInstances != 0)
		    heading /= numInstances;
		
		// clear all the arraylists
		accx.clear();
		accy.clear();
		accz.clear();
		orientx.clear();
		orienty.clear();
		orientz.clear();
		
		// if there are overflows
		for (int i = 0; i < overflow.size(); i += 6) {
			accx.add(overflow.get(i));
			accy.add(overflow.get(i+1));
			accz.add(overflow.get(i+2));
			orientx.add(overflow.get(i+3));
			orienty.add(overflow.get(i+4));
			orientz.add(overflow.get(i+5));
		}
		
		overflow.clear();
		
		// and reset the timers
		start = -1;
		curr = -1;
		
		mutex.release();
		return new DevicePose(pos[0], pos[1], pos[2], heading);
	}

	public void startTimer() {
		start = curr = Calendar.getInstance().getTimeInMillis();
	}
}
