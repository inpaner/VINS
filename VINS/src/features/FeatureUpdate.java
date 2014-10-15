package features;

import java.util.ArrayList;
import java.util.List;

import ekf.PointDouble;

public class FeatureUpdate {
	private List<Integer> badPointsIndex;
	private List<PointDouble> currentPoints;
	private List<PointDouble> newPoints;
	
	public FeatureUpdate(){
		badPointsIndex = new ArrayList<>();
		currentPoints = new ArrayList<>();
		newPoints = new ArrayList<>();
	}
	
	public List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}
	
	public List<PointDouble> getCurrentPoints() {
		return currentPoints;
	}
	
	public List<PointDouble> getNewPoints() {
		return newPoints;
	}
	
	void setBadPointsIndex(List<Integer> badPointsIndex){
		this.badPointsIndex = badPointsIndex;
	}
	
	void setCurrentPoints(List<PointDouble> currentPoints){
		this.currentPoints = currentPoints;
	}
	
	void setNewPoints(List<PointDouble> newPoints){
		this.newPoints = newPoints;
	}
}
