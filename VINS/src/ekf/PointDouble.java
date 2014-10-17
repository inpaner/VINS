package ekf;

public class PointDouble {

	private double x;
	private double y;

	public PointDouble(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double computeDistanceTo(PointDouble other) {
		if (other == null)
			return -1;

		return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
	}

	/**
	 * Take note that this will compute the angle with respect to the POSITIVE
	 * X-AXIS.
	 **/
	public double computeRadiansTo(PointDouble other) {

		double angle = Math.atan((y - other.y) / (x - other.x));

		// angle is only from -90 to 90, so have to adjust if the other point is
		// to the left of this point

		if (other.x < x)
			angle += Math.PI;

		return angle;
	}

	public PointDouble add(double offsetX, double offsetY) {
		return new PointDouble(x + offsetX, y + offsetY);
	}

}
