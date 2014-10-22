package logger;
public class INSDataObject extends DataObject {
	public INSDataObject (double aX, double aY, double aZ, double gX, double gY, double gZ) {
		contents[0] = aX + "";
		contents[1] = aY + "";
		contents[2] = aZ + "";
		contents[3] = gX + "";
		contents[4] = gY + "";
		contents[5] = gZ + "";
	}
}
