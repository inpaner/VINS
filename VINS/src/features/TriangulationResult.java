package features;

import java.util.List;

import ekf.PointDouble;

class TriangulationResult {
	private List<PointDouble> currentFeatures;
	private List<PointDouble> newFeatures;
	
	TriangulationResult(List<PointDouble> currentFeatures, List<PointDouble> newFeatures) {
		this.currentFeatures = currentFeatures;
		this.newFeatures = newFeatures;
	}

	public List<PointDouble> getCurrentPoints() {
		return currentFeatures;
	}

	public List<PointDouble> getNewFeatures() {
		return newFeatures;
	}
	
	
}
