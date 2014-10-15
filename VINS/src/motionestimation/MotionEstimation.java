package motionestimation;

public interface MotionEstimation {
	/** Inputs data into the estimation engine, in a specific format.
	 	How the data is formatted will be up to the implementator
	**/
	public abstract void inputData(SensorEntry s);
	
	/** Heading returned is an estimate of the current heading, in radians
	 	0 radians implies that the phone at (0, 0) is facing (1, 0)
	
	 	Displacement returned is an estimate of the current displacement, in meters(m)
	**/
	public abstract DevicePose getHeadingAndDisplacement();

	/** starts timer, for the 3Hz thing **/
	public abstract void startTimer();
}