package features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.MatOfPoint2f;

class OpticalFlowResult {
	private MatOfPoint2f nearFeatures;
	private MatOfPoint2f farFeatures;
	private List<Integer> badPointsIndex;
	
	private double currentSize;
	
	OpticalFlowResult(MatOfPoint2f nearFeatures, MatOfPoint2f farFeatures, List<Integer> badPointsIndex, double currentSize) {
		this.nearFeatures = nearFeatures;
		this.farFeatures = farFeatures;
		this.currentSize = currentSize;
		this.badPointsIndex = badPointsIndex;
	}

	MatOfPoint2f getNearFeatures() {
		return nearFeatures;
	}

	MatOfPoint2f getFarFeatures() {
		return farFeatures;
	}
	
	double getCurrentSize() {
		return currentSize;
	}
	
	List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}
	
	boolean isNotEmpty() {
		return !nearFeatures.empty() && !farFeatures.empty();
	}
	
}
