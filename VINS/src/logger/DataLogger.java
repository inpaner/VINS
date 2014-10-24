package logger;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

public class DataLogger {
	private static boolean logging = false;
	private static FileWriter fw;
	private static FileWriter consolidate;
	
	public static void startLogging(String prefix) {
		try {
			if (consolidate == null)
				consolidate = new FileWriter("Consolidate_Log.txt");
			if (fw == null)
				fw = new FileWriter(prefix + "_Log.txt");
		} catch (IOException e) {

			e.printStackTrace();
			return;
		}
		
		logging = true;
	}
	
	public static void logData(DataObject data) {
		try {
			fw.append(Calendar.getInstance().getTimeInMillis() + " " + data);
			consolidate.append(Calendar.getInstance().getTimeInMillis() + " " + data);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static void stopLogging() {
		if (!logging)
			return;
		
		try {
			fw.flush();
			fw.close();
			
			consolidate.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		fw = null;
	}
	
}