package logger;
public abstract class DataObject {
	public String[] contents;
	
	public String toString() {
		String out = contents[0];
		
		for (int i = 1; i < contents.length; i++)
			out += " " + contents[i];
		
		return out;
	}
}
