package motionestimation;

import java.util.ArrayList;
import java.util.Calendar;

public class IntegrateMotionEstimation implements MotionEstimation {
	// timer counters
	long start = -1;
	long curr = -1;
	
	// accelerometer data
	ArrayList<Double> accx = new ArrayList<Double>();
	ArrayList<Double> accy = new ArrayList<Double>();
	ArrayList<Double> accz = new ArrayList<Double>();
	
	// gyroscope data
	ArrayList<Double> gyrx = new ArrayList<Double>();
	ArrayList<Double> gyry = new ArrayList<Double>();
	ArrayList<Double> gyrz = new ArrayList<Double>();
	
	// overflow data, add when next time
	ArrayList<Double> overflow = new ArrayList<Double>();
	
	// format is [accx y z gyrox y z]
	public void inputData(SensorEntry s) {
		if (start == -1)
			startTimer();
		else
			curr = Calendar.getInstance().getTimeInMillis();
		
		if (start + 333 >= curr) {
			accx.add(s.getAcc_x());
			accy.add(s.getAcc_y());
			accz.add(s.getAcc_z());
			
			gyrx.add(s.getGyro_x());
			gyry.add(s.getGyro_y());
			gyrz.add(s.getGyro_z());
		}
		else {
			overflow.add(s.getAcc_x());
			overflow.add(s.getAcc_y());
			overflow.add(s.getAcc_z());
			overflow.add(s.getGyro_x());
			overflow.add(s.getGyro_y());
			overflow.add(s.getGyro_z());
		}
	}
	
	public DevicePose getHeadingAndDisplacement() {
		// D O U B L E I N T E G R A T I O N
		double[] pos = new double[3];
		
		// x = a*t^2/4
		for (Double d : accx)
			pos[0] += d;
		for (Double d : accy)
			pos[1] += d;
		for (Double d : accz)
			pos[2] += d;
		
		pos[0] *= (curr-start)*(curr-start)/4000.0;
		pos[1] *= (curr-start)*(curr-start)/4000.0;
		pos[2] *= (curr-start)*(curr-start)/4000.0;
		
		// average all the gyroscope readings
		int numInstances = gyrx.size();
		double heading = 0;
		for (Double d : gyrx)
			heading += d;
		
		if(numInstances != 0)
		    heading /= numInstances;
		
		// clear all the arraylists
		accx.clear();
		accy.clear();
		accz.clear();
		gyrx.clear();
		gyry.clear();
		gyrz.clear();
		
		// if there are overflows
		for (int i = 0; i < overflow.size(); i += 6) {
			accx.add(overflow.get(i));
			accy.add(overflow.get(i+1));
			accz.add(overflow.get(i+2));
			gyrx.add(overflow.get(i+3));
			gyry.add(overflow.get(i+4));
			gyrz.add(overflow.get(i+5));
		}
		
		overflow.clear();
		
		// and reset the timers
		start = -1;
		curr = -1;
		
		return new DevicePose(pos[0], pos[1], pos[2], heading);
	}

	public void startTimer() {
		start = curr = Calendar.getInstance().getTimeInMillis();
	}
}
