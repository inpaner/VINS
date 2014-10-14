package features;

import java.util.ArrayList;
import java.util.List;

import ekf.PointDouble;

public class FeatureUpdate {
	List<Integer> badPointsIndex = new ArrayList<>();
	List<PointDouble> currentPoints = new ArrayList<>();
	List<PointDouble> newPoints = new ArrayList<>();
	
	public List<PointDouble> getNewPoints() {
		return newPoints;
	}
	
	public List<PointDouble> getCurrentPoints() {
		return currentPoints;
	}
	
	public List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}
}
