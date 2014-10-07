package features;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import android.app.Service;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.IBinder;
import android.util.Log;
import android.widget.Toast;

public class FeatureService extends Service {
	private static final String TAG = "FeatureService";
	private boolean stopThread;
	private Thread thread;
	private VideoCapture camera;
	private int frameWidth;
	private int frameHeight;
	private int cameraIndex = 0;
	private Bitmap cacheBitmap;
	
	static {
	    if (!OpenCVLoader.initDebug()) {
	        Log.e("Driver", "Debug error shit");
	    }
	}
	
	@Override
	public IBinder onBind(Intent intent) {
		return null;
	}

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				Log.i(TAG, "OpenCV loaded successfully");
				try {
					if (!connectCamera(320, 240))
						Log.e(TAG, "Could not connect camera");
					else
						Log.d(TAG, "Camera successfully connected");
				} catch (Exception e) {
					Log.e(TAG,
							"MyServer.connectCamera throws an exception: "
									+ e.getMessage());
				}

				Toast.makeText(FeatureService.this, "service started", Toast.LENGTH_LONG).show();
				Log.d(TAG, "service.onStart: end");
				
			} break;

			default: {
				super.onManagerConnected(status);
			} break;
			
			}
		}
	};

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		Log.d(TAG, "service.onStart: begin");
		if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this,
				mLoaderCallback)) {
			Log.e(TAG, "Cannot connect to OpenCV Manager");
		}
		else {
			Log.d(TAG, "Successfully loaded opencv");
		}
		
		return Service.START_NOT_STICKY;
	}
	
	@Override
	public void onDestroy() {

		this.disconnectCamera();

		Toast.makeText(this, "service stopped", Toast.LENGTH_LONG).show();
		Log.d(TAG, "onDestroy");
	}

	@Override
	public void onStart(Intent intent, int startid) {
		Log.d(TAG, "service.onStart: begin");
		
		
	}

	private boolean connectCamera(int width, int height) {
		/* First step - initialize camera connection */

		if (!initializeCamera(width, height))
			return false;
		
		/* now we can start update thread */
		thread = new Thread(new CameraWorker());
		thread.start();

		return true;
	}

	private boolean initializeCamera(int width, int height) {
		synchronized (this) {
			
			camera = new VideoCapture(Highgui.CV_CAP_ANDROID_BACK);
			
			if (camera == null)
				return false;
			Log.d(TAG, "camera not null");
			
			if (camera.isOpened() == false)
				return false;
			Log.d(TAG, "camera opened");
			
			// java.util.List<Size> sizes = mCamera.getSupportedPreviewSizes();

			/*
			 * Select the size that fits surface considering maximum size
			 * allowed
			 */
			Size frameSize = new Size(width, height);

			frameWidth = (int) frameSize.width;
			frameHeight = (int) frameSize.height;

			allocateCache();

			camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, frameSize.width);
			camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, frameSize.height);
		}

		Log.i(TAG, "Selected camera frame size = (" + frameWidth + ", "
				+ frameHeight + ")");

		return true;
	}

	protected void allocateCache() {
		cacheBitmap = Bitmap.createBitmap(frameWidth, frameHeight,
				Bitmap.Config.ARGB_8888);
	}

	private void releaseCamera() {
		synchronized (this) {
			if (camera != null) {
				camera.release();
			}
		}
	}

	private void disconnectCamera() {
		/*
		 * 1. We need to stop thread which updating the frames 2. Stop camera
		 * and release it
		 */
		try {
			stopThread = true;
			thread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} finally {
			thread = null;
			stopThread = false;
		}

		/* Now release camera */
		releaseCamera();
	}

	protected void deliverAndDrawFrame(NativeCameraFrame frame) {
		Mat modified = frame.gray();

		boolean bmpValid = true;
		if (modified != null) {
			try {
				Utils.matToBitmap(modified, cacheBitmap);
			} catch (Exception e) {
				Log.e(TAG, "Mat type: " + modified);
				Log.e(TAG, "Bitmap type: " + cacheBitmap.getWidth() + "*"
						+ cacheBitmap.getHeight());
				Log.e(TAG,
						"Utils.matToBitmap() throws an exception: "
								+ e.getMessage());
				bmpValid = false;
			}
		}
	}

	private class NativeCameraFrame {
		public Mat rgba() {
			capture.retrieve(mRgba, Highgui.CV_CAP_ANDROID_COLOR_FRAME_RGBA);
			return mRgba;
		}

		public Mat gray() {
			capture.retrieve(mGray, Highgui.CV_CAP_ANDROID_GREY_FRAME);
			return mGray;
		}

		public NativeCameraFrame(VideoCapture capture) {
			this.capture = capture;
			mGray = new Mat();
			mRgba = new Mat();
		}

		private VideoCapture capture;
		private Mat mRgba;
		private Mat mGray;
	};
	
	private class InitializeCamera implements Runnable {

		@Override
		public void run() {

		}
		
	}
	
	private class CameraWorker implements Runnable {
		public void run() {
			do {
				if (!camera.grab()) {
					Log.e(TAG, "Camera frame grab failed");
					break;
				}

				deliverAndDrawFrame(new NativeCameraFrame(camera));

			} while (!stopThread);
		}
	}
}
