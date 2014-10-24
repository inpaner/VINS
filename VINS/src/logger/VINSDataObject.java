package logger;

import java.util.List;

import features.FeatureUpdate;
import ekf.PointDouble;

public class VINSDataObject extends DataObject {
	public VINSDataObject (FeatureUpdate a) {
		List<Integer> bpi = a.getBadPointsIndex();
		List<PointDouble> currPoints = a.getCurrentPoints();
		List<PointDouble> newPoints = a.getNewPoints();
		
		contents = new String[3 + bpi.size() + currPoints.size() + newPoints.size()];
		
		int ctr = 0;
		contents[ctr] = bpi.size() + "\\n";
		ctr++;
		
		for (int i = 0; i < bpi.size(); i++, ctr++)
			contents[ctr] = bpi.get(i) + "";
		
		contents[ctr] = currPoints.size() + "\\n";
		ctr++;
		
		for (int i = 0; i < currPoints.size(); i++, ctr++) 
			contents[ctr] = currPoints.get(i).toString();
		
		contents[ctr] = newPoints.size() + "\\n";
		ctr++;
		
		for (int i = 0; i < newPoints.size(); i++, ctr++)
			contents[ctr] = newPoints.get(i).toString();
	}
}
