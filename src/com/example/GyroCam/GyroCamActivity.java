package com.example.GyroCam;

import java.util.Iterator;
import java.text.DecimalFormat;

import com.example.GyroCam.GyroCamView;

import android.app.Activity;
import android.os.Bundle;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.widget.CheckBox;
import android.widget.RelativeLayout;
import android.widget.Switch;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.hardware.SensorEventListener;
import android.location.Location;
import android.location.LocationManager;
import android.location.LocationListener;
import android.location.GpsStatus;
import android.location.GpsSatellite;

public class GyroCamActivity extends Activity {
	private View mainView;
	private GyroCamView gyroCamView;
	public static final String TAG = "Gyro_Cam";
	public static final String PREFS_NAME = "GyroCamPrefs";
	
	// UI declarations
	private int screenOrientation;
	private boolean mKeepScreenOn;
	private boolean mKeepScreenLandscape;
	
	private CheckBox screenCheckBox;
	private CheckBox orientationCheckBox;
	private Switch gpsSwitch;
	private ProgressBar gpsProgressBar;
	private TextView gpsTextSats;
	private TextView gpsTextStatus;
	private TextView gpsCoordinates;
	private TextView tvAngleAcc;
	private TextView tvAngleGyr;
	private TextView tvAngleKal;
	
	// sensors declarations
	private SensorManager mSensorManager;
	private SensorEventListener mSensorEventListener;
	private Sensor accSensor, gyrSensor, magSensor;
	private LocationManager mLocationManager;
	private LocationListener mLocationListener;
	private GpsStatus.Listener mGpsStatusListener;
	
	private float locLatitude = 0;
	private float locLongitude = 0;
	private float locAccuracy = 0;
	private float locSpeed = 0;
	private boolean locFix = false;
	private int locSats;
	private int locSatsFix;
	
	private float dts, dtd;
	private long lastsense, lastdraw, lastwrite;
	private int accUpdated = 0;
	private int gyrUpdated = 0;

	private class sensval {
		  public double x, y, z;
		  public float dt;
	}

	private sensval accVal = new sensval();
	private sensval gyrVal = new sensval();
	private sensval magVal = new sensval();
	private sensval angleaVal = new sensval();
	private sensval anglegVal = new sensval();
	private sensval anglekVal = new sensval();
	public float angleLean;
	
	static double qAngleInit = 0.001; //process covariance noise of accelerometer
	static double qGyroInit = 0.005;  //process covariance noise of gyroscope
	static double rAngleInit = 3000.0;  //measurement covariance noise

	KalmanFilter filterRoll = new KalmanFilter(qAngleInit, qGyroInit, rAngleInit);
	KalmanFilter filterPitch = new KalmanFilter(qAngleInit, qGyroInit, rAngleInit);
	KalmanFilter filterYaw = new KalmanFilter(qAngleInit, qGyroInit, rAngleInit);
		
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate() Called");
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        
        setContentView(R.layout.main); 
        RelativeLayout layout = (RelativeLayout) findViewById(R.id.layout_draw);
        gyroCamView = new GyroCamView(this);
        layout.addView(gyroCamView);
        
        screenCheckBox = (CheckBox) findViewById(R.id.cb_ScreenUse);
        orientationCheckBox = (CheckBox) findViewById(R.id.cb_ScreenLandscape);
        gpsSwitch = (Switch) findViewById(R.id.sw_UseGps);
        gpsProgressBar = (ProgressBar) findViewById(R.id.pb_ScanGps);
        gpsTextSats = (TextView) findViewById(R.id.tv_GpsSats);
        gpsTextStatus = (TextView) findViewById(R.id.tv_GpsStatus);
        gpsCoordinates = (TextView) findViewById(R.id.tv_GpsCoord);
        tvAngleAcc = (TextView) findViewById(R.id.tv_AngleAcc);
        tvAngleGyr = (TextView) findViewById(R.id.tv_AngleGyr);
        tvAngleKal = (TextView) findViewById(R.id.tv_AngleKal);
        
        // update GUI
        gpsProgressBar.setVisibility(View.INVISIBLE);
        gpsTextSats.setVisibility(View.INVISIBLE);
        gpsTextStatus.setVisibility(View.INVISIBLE);
        // read saved settings
        SharedPreferences settings = getSharedPreferences(PREFS_NAME, 0);
        mKeepScreenOn = settings.getBoolean("keepScreenOn", false);
        mKeepScreenLandscape = settings.getBoolean("keepScreenLandscape", false);
        
        mainView = findViewById(R.id.layout_main);
        if (mKeepScreenOn) {
    		mainView.setKeepScreenOn(true);
    		screenCheckBox.setChecked(true);
        } else {
        	mainView.setKeepScreenOn(false);
        	screenCheckBox.setChecked(false);
        }
        if (mKeepScreenLandscape) {
        	setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        	orientationCheckBox.setChecked(true);
        } else {
        	setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR);
        	orientationCheckBox.setChecked(false);
        }
        
        
        // setup some SensorListeners
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorEventListener = new MySensorEventListener();
        accSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyrSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorManager.registerListener(mSensorEventListener, accSensor, SensorManager.SENSOR_DELAY_UI); //GAME);
        mSensorManager.registerListener(mSensorEventListener, gyrSensor, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(mSensorEventListener, magSensor, SensorManager.SENSOR_DELAY_UI);
        // setup the LocationListener
        mLocationManager = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
        mLocationListener = new MyLocationListener();
        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, mLocationListener);
        mGpsStatusListener = new MyGpsStatusListener();
        mLocationManager.addGpsStatusListener(mGpsStatusListener);
	}
    
    @Override
	protected void onPause() {
		super.onPause();
		//gyroCamView.pause();
		SharedPreferences settings = getSharedPreferences(PREFS_NAME, 0);
		SharedPreferences.Editor editor = settings.edit();
		editor.putBoolean("keepScreenOn", mKeepScreenOn);
		editor.putBoolean("keepScreenLandscape", mKeepScreenLandscape);
		editor.commit();
	}

	@Override
	protected void onResume() {
		super.onResume();
		//gyroCamView.resume();
	}
	
	@Override
	protected void onStop() {
		super.onStop();

	}
	
	@Override
	public void onConfigurationChanged(Configuration newConfig) {
		Log.d(TAG, "onConfigurationChanged() Called "+String.valueOf(newConfig.orientation));
		screenOrientation = newConfig.orientation;
		super.onConfigurationChanged(newConfig);
		resetKalman();
	}
	
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
    	switch (item.getItemId()) {
    		case R.id.menu_exit:
    			this.finish();
    			return true;
    		case R.id.menu_settings:
    			
    			return true;
    		default:
    			return super.onOptionsItemSelected(item);
    	}
    		
    }
    
//	public class GyroCamView extends SurfaceView implements Runnable {
//
//		Thread t = null;
//		SurfaceHolder holder;
//		boolean isItOK = false;
//		
//		public GyroCamView(Context context) {
//			super(context);
//			// TODO Auto-generated constructor stub
//			holder = getHolder();
//		}
//
//		@Override
//		public void run() {
//			// TODO Auto-generated method stub
//			while (isItOK == true) {
//				if (!holder.getSurface().isValid()) continue;
//				Canvas c = holder.lockCanvas();
//				c.drawARGB(128, 128, 128, 128);
//				Paint paint = new Paint();
//				paint.setStyle(Paint.Style.STROKE);
//				int center_x = 200;
//				int center_y = 200;
//				int radius = 5;
//				RectF rect = new RectF();
//				rect.set(center_x - radius, center_y - radius, center_x + radius, center_y + radius);
//				c.drawRoundRect(rect, 100, 50, paint);
//				holder.unlockCanvasAndPost(c);
//			}
//		}
//		
//		public void pause() {
//			isItOK = false;
//			while (true) {
//				try {
//					t.join();
//				}
//				catch (InterruptedException e) {
//					e.printStackTrace();
//				}
//				break;
//			}
//			t = null;
//		}
//		
//		public void resume() {
//			isItOK = true;
//			t = new Thread(this);
//			t.start();
//		}
//    	
//    }
    
    /** Sensors */
	// Setup our SensorEventListener
    private class MySensorEventListener implements SensorEventListener {
    	public void onSensorChanged(SensorEvent event) {
			int eventType = event.sensor.getType();
			if(eventType == Sensor.TYPE_ACCELEROMETER) {
				accVal.x = event.values[0];
				accVal.y = event.values[1];
				accVal.z = event.values[2];
				accUpdated = 1;
			}
			if(eventType == Sensor.TYPE_GYROSCOPE) {
				gyrVal.x = event.values[0];
				gyrVal.y = event.values[1];
				gyrVal.z = event.values[2];
				gyrUpdated = 1;
			}
			if(eventType == Sensor.TYPE_MAGNETIC_FIELD) {
				magVal.x = event.values[0];
				magVal.y = event.values[1];
				magVal.z = event.values[2];
				if ((accUpdated == 1) && (gyrUpdated == 1)) {
					accUpdated = 0;
					gyrUpdated = 0;
					updateSensors();
					gyroCamView.updateDraw((float)angleaVal.z, (float)anglegVal.z, (float)anglekVal.z);
				}
			}
    	}
    	public void onAccuracyChanged(Sensor sensor, int accuracy) {
    		// do nothin...
    	}
    }
    
	private void updateSensors() {
		dts = (System.nanoTime() - lastsense)/1000000;
		lastsense = System.nanoTime();
		accVal.dt = dts;
		gyrVal.dt = dts;
		magVal.dt = dts;	
		if (screenOrientation == Configuration.ORIENTATION_PORTRAIT) {
			Log.d(TAG, "updateSensors " + String.valueOf(dts) + " - portrait");
			angleaVal.x = Math.atan2(accVal.x, accVal.z) - Math.PI/2;
			angleaVal.y = Math.atan2(magVal.y, magVal.z); 				//Magnetfeld
			angleaVal.z = -Math.atan2(accVal.y, accVal.x) + Math.PI/2;
		} else {
			Log.d(TAG, "updateSensors " + String.valueOf(dts) + " - landscape");
			angleaVal.x = Math.atan2(accVal.y, accVal.z) - Math.PI/2;
			angleaVal.y = Math.atan2(magVal.x, magVal.z); 				//Magnetfeld
			angleaVal.z = Math.atan2(accVal.x, accVal.y) - Math.PI/2;	
		}
		
		double qx = gyrVal.x;
		double qy = gyrVal.y;
		double qz = gyrVal.z;
		anglegVal.x += qx * gyrVal.dt / 1000;
		anglegVal.y += qy * gyrVal.dt / 1000;
		anglegVal.z += qz * gyrVal.dt / 1000;
		filterPitch.kalmanPredict(gyrVal.x, dts/1000); 
		filterPitch.kalmanNanCheck();
		anglekVal.x = filterPitch.kalmanUpdate(angleaVal.x);
		filterYaw.kalmanPredict(gyrVal.y, dts/1000);
		filterYaw.kalmanNanCheck();
		anglekVal.y = filterYaw.kalmanUpdate(angleaVal.y);
		filterRoll.kalmanPredict(gyrVal.z, dts/1000);
		filterRoll.kalmanNanCheck();
		anglekVal.z = filterRoll.kalmanUpdate(angleaVal.z);
		angleLean = (float)anglekVal.z;
		
		String pattern = "0.000";
		DecimalFormat mFormatter = new DecimalFormat(pattern);

		tvAngleAcc.setText("angle acc: " + mFormatter.format(angleaVal.z));
		tvAngleGyr.setText("angle gyr: " + mFormatter.format(anglegVal.z));
		tvAngleKal.setText("angle kal: " + mFormatter.format(anglekVal.z));
	}
	
	private void resetKalman() {
		filterRoll.kalmanReset();
		filterPitch.kalmanReset();
		filterYaw.kalmanReset();
		anglegVal.x = 0; anglegVal.y = 0; anglegVal.z = 0; //-PI/2;  
		anglekVal.x = 0; anglekVal.y = 0; anglekVal.z = 0; //-PI/2;
	}
	
	/*
	 * Kalman filtering
	 */
	private class KalmanFilter {
		/* These variables represent our state matrix x */
		private double xAngle, xBias;
		/* Our error covariance matrix */
		private double P00, P01, P10, P11;	
		  	
		/* 
		 * Q is a 2x2 matrix of the covariance. Because we
		 * assuma the gyro and accelero noise to be independend
		 * of eachother, the covariances on the / diagonal are 0.
		 *
		 * Covariance Q, the process noise, from the assumption
		 *    x = F x + B u + w
		 * with w having a normal distribution with covariance Q.
		 * (covariance = E[ (X - E[X])*(X - E[X])' ]
		 * We assume is linair with dt
		 */
		private double qAngle, qGyro;
		/*
		 * Covariance R, our observation noise (from the accelerometer)
		 * Also assumed to be liniar with dt
		 */
		private double rAngle;
		
		KalmanFilter(double mQangle, double mQgyro, double mRangle) {
			qAngle = mQangle;
			qGyro  = mQgyro;
			rAngle = mRangle;
		}
		
		public void kalmanReset() {
			xAngle = 0;
			xBias = 0;
			
		}
		
		public void kalmanPredict(double dotAngle, double dt) {
		  xAngle += dt * (dotAngle - xBias);
		  P00 +=  - dt * (P10 + P01) + qAngle * dt;
		  P01 +=  - dt * P11;
		  P10 +=  - dt * P11;
		  P11 +=  + qGyro * dt;
		}
		
		/*!
		 *  Sometimes (during initialization?) we get NaN (not a number). This routine
		 *  tries to get the calculations back on track if something like that happens.
		 */
		public void kalmanNanCheck() {
			if (xAngle != xAngle)  //Check for NaN
			{
				xAngle = 0;
				//printf ("NaN detected!!!!\n\r");
				if (xBias != xBias)
				{
					xBias = 0.0;
					P00 = 1.0;
					P01 = 0.0;
					P10 = 0.0;
					P11 = 1.0;
				}
			}
		}
		
		public double kalmanUpdate(double angleM) {
			double y = angleM - xAngle;
			double S = P00 + rAngle;
			double K0 = P00 / S;
			double K1 = P10 / S;

			xAngle +=  K0 * y;
			xBias  +=  K1 * y;
			P00 -= K0 * P00;
			P01 -= K0 * P01;
			P10 -= K1 * P00;
			P11 -= K1 * P01;

			return xAngle;
		}
		
	}
	
    // Setup our LocationListener
    private class MyLocationListener implements LocationListener {
    	public void onLocationChanged(Location argLocation) {
    		locLatitude = (float)argLocation.getLatitude();
    		locLongitude = (float)argLocation.getLongitude();
    	    locAccuracy = (float)argLocation.getAccuracy(); // accuracy of fix in m
    	    locSpeed = (float)argLocation.getSpeed(); // speed in m/s
    		gpsCoordinates.setText(String.format("%.9f° - %.9f° - %.2fm/s", locLatitude, locLongitude, locSpeed));
    	}
    	public void onProviderDisabled(String provider) {
    		// do nothing...
    	}
    	public void onProviderEnabled(String provider) {
    		// do nothing...
    	}
    	public void onStatusChanged(String provider, int event, Bundle extras) {
    		// do nothing...
    	}
    }
    
//	 Setup our GpsStatusListener
    private class MyGpsStatusListener implements GpsStatus.Listener {
    	public void onGpsStatusChanged(int event) {
    		switch(event) {
    			case GpsStatus.GPS_EVENT_SATELLITE_STATUS:
    				GpsStatus gpsStat = mLocationManager.getGpsStatus(null);
    				Iterable<GpsSatellite> sats = gpsStat.getSatellites();
    				Iterator satI = sats.iterator();
    				int countFix = 0;
    				int count = 0;
    				while(satI.hasNext()) {
    					count++;
    					GpsSatellite gpsSat = (GpsSatellite)satI.next();
    					if (gpsSat.usedInFix()) {
    						countFix++;
    					}
    				}
    				locSats = count;
    				locSatsFix = countFix;
    				gpsTextSats.setText(String.format("%1d/%1d",locSats,locSatsFix));
    				break;
    			case GpsStatus.GPS_EVENT_FIRST_FIX:
    				gpsTextStatus.setText("fixed");
    				gpsProgressBar.setVisibility(View.GONE);
    				locFix = true;
    				break;
    			case GpsStatus.GPS_EVENT_STARTED:
    				gpsTextStatus.setText("scanning...");
    				gpsProgressBar.setVisibility(View.VISIBLE);
    				locFix = false;
    				break;
    			case GpsStatus.GPS_EVENT_STOPPED:
    				gpsTextStatus.setText("stopped");
    				gpsSwitch.setChecked(false);
    				locFix = false;
    				break;
    		}
    	}  
	}
    
    /** UI-stuff Callback */
    public void uiKeepScreenOn(View view) {
    	if (screenCheckBox.isChecked()) {
    		mKeepScreenOn = true;
    		view.setKeepScreenOn(true);
    	} else {
    		mKeepScreenOn = false;
    		view.setKeepScreenOn(false);
    	}
    }
    
    public void uiKeepScreenLandscape(View view) {
    	if (orientationCheckBox.isChecked()) {
    		mKeepScreenLandscape = true;
    		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
    	} else {
    		mKeepScreenLandscape = false;
    		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR);
    	}
    }
    
    public void uiActivateGps(View view) {
    	if (gpsSwitch.isChecked()) {
	        gpsProgressBar.setVisibility(View.VISIBLE);
	        gpsTextSats.setVisibility(View.VISIBLE);
	        gpsTextStatus.setVisibility(View.VISIBLE);
    	} else {
            gpsProgressBar.setVisibility(View.INVISIBLE);
            gpsTextSats.setVisibility(View.INVISIBLE);
            gpsTextStatus.setVisibility(View.INVISIBLE);
    	}
    }
    
    public void uiResetKalman(View view) {
    	resetKalman();
    }

}