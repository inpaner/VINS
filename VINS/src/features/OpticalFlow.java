package features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.video.Video;

import android.util.Log;

class OpticalFlow {
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);
	private FeatureDetector detector = FeatureDetector.create(FeatureDetector.FAST);

	OpticalFlowResult getFeatures(Mat checkpointImage, Mat nearImage, Mat farImage, MatOfPoint2f checkpointFeatures) {
		
		// Checkpoint to near frame
		
		MatOfPoint2f nearFeatures = new MatOfPoint2f();
		MatOfByte cpNearStatus = new MatOfByte();
		MatOfFloat cpNearError = new MatOfFloat();
		
		Mat detectMask = nearImage.clone();
		detectMask.setTo(WHITE);
		List<Point> nearFeaturesList = new ArrayList<>();
		
		boolean hasCurrent = false;
		double currentSize = checkpointFeatures.size().height;
		
		if (checkpointFeatures.size().height > 0) {
			hasCurrent = true;
			Video.calcOpticalFlowPyrLK(checkpointImage, nearImage, checkpointFeatures, nearFeatures, cpNearStatus, cpNearError);
			nearFeaturesList = nearFeatures.toList();
			
			// draw mask for detection
			int index = 0;
			for (Byte item : cpNearStatus.toList()) {
				if (item.intValue() == 1) {
					Core.circle(detectMask, nearFeaturesList.get(index), 10, BLACK, -1);
				}
				index++;
			}
		}
		
		// detect new features
		
		MatOfKeyPoint rawNearNewFeatures = new MatOfKeyPoint();
		MatOfPoint2f nearNewFeatures = new MatOfPoint2f();
		detector.detect(nearImage, rawNearNewFeatures, detectMask);
		if (rawNearNewFeatures.size().height > 0) {
			nearNewFeatures = convert(rawNearNewFeatures);
		}
		
		//// Near frame to far frame
		
		nearFeatures.push_back(nearNewFeatures);
		nearFeaturesList = nearFeatures.toList(); 
		
		MatOfPoint2f farFeatures = new MatOfPoint2f();
		MatOfByte nearFarStatus = new MatOfByte();
		MatOfFloat nearFarError = new MatOfFloat();
					
		if (nearFeatures.size().height > 0) {
			Video.calcOpticalFlowPyrLK(nearImage, farImage, nearFeatures, farFeatures, nearFarStatus, nearFarError);
		}
		
		List<Point> farFeaturesList = farFeatures.toList();
		List<Point> goodNearFeaturesList = new ArrayList<>();
		List<Point> goodFarFeaturesList = new ArrayList<>();
		List<Integer> badPointsIndex = new ArrayList<>();
		
		// Find good features, bad features index
		
		int index = 0;
		
		List<Byte> cpNearStatusList = null;
		if (hasCurrent) {
			cpNearStatusList = cpNearStatus.toList();
		}
		
		if (nearFarStatus.size().height > 0) {
			for (Byte firstStatus : nearFarStatus.toList()) {
				boolean isGood = false;
				if (index < currentSize) {
					Byte secondStatus = cpNearStatusList.get(index);
					if ((firstStatus.intValue() & secondStatus.intValue()) == 1) {  
						isGood = true;
					} else {
						badPointsIndex.add(Integer.valueOf(index));
					}
				} else {
					isGood = true;
				}
				
				if (isGood) {
					goodNearFeaturesList.add( nearFeaturesList.get(index) );
					goodFarFeaturesList.add( farFeaturesList.get(index) );
				} 
				index++;
			}
		}
		
		MatOfPoint2f goodNearFeatures = new MatOfPoint2f();
		MatOfPoint2f goodFarFeatures = new MatOfPoint2f();
		goodNearFeatures.fromList(goodNearFeaturesList);
		goodFarFeatures.fromList(goodFarFeaturesList);
		
		OpticalFlowResult result = new OpticalFlowResult(goodNearFeatures, goodFarFeatures, badPointsIndex, currentSize);
		return result;
	}
	
	private MatOfPoint2f convert(MatOfKeyPoint keyPoints) {
		KeyPoint[] keyPointsArray = keyPoints.toArray();
		Point[] pointsArray = new Point[keyPointsArray.length];

		for (int i = 0; i < keyPointsArray.length; i++) {
			pointsArray[i] = (Point) keyPointsArray[i].pt;
		}

		return new MatOfPoint2f(pointsArray);
	}
}
