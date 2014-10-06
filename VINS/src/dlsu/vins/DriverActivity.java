package dlsu.vins;

import android.app.Activity;
import android.os.Bundle;

public class DriverActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
	super.onCreate(savedInstanceState);
	setContentView(R.layout.activity_driver);
	
	new DriverThread().start();
    }
    
}
