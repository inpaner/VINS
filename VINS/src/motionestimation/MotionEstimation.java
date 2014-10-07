package motionestimation;

public interface MotionEstimation {
	// inputs data into the estimation engine, in a specific format
	// how the data is formatted, bahala na si implementator
	public abstract void inputData(SensorEntry s);
	
	// heading returned is an estimate of the current heading, in radians
	// 0 radians implies that the phone at (0, 0) is facing (1, 0)
	
	// displacement returned is an estimate of the current displacement, in m
	public abstract DevicePose getHeadingAndDisplacement();

	// starts timer, for the 3Hz thing
	public abstract void startTimer();
}