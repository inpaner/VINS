package ekf;

public class PointDouble {

    private double x;
    private double y;
    
    public PointDouble(double x, double y){
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
    
    public double computeDistanceTo(PointDouble other){
	if(other == null)
	    return -1;
	
	return Math.sqrt(Math.pow(x-other.x,2) + Math.pow(y-other.y,2));
    }
    
    
}
