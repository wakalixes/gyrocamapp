package com.example.GyroCam;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.View;

public class GyroCamView extends View {
	Paint paint = new Paint();
	
	private float angleAcc, angleGyro, angleKal;
    private int linewidth = 4;
    private int linelength = 400;
    
	private int i = 0;
	
	public GyroCamView(Context context) {
		super(context);
		
	}
	
	public void onDraw(Canvas canvas) {
		super.onDraw(canvas);
		
        paint.setAntiAlias(true);
        i+=10;
        if (i>canvas.getWidth()) i = 0;
        
        int xcenter = canvas.getWidth()/2;
        int ycenter = canvas.getHeight()/2;
        
        paint.setColor(Color.BLUE);
        drawAngle(canvas, angleAcc);
        paint.setColor(Color.GREEN);
        drawAngle(canvas, angleGyro);
        paint.setColor(Color.RED);
        drawAngle(canvas, angleKal);
        canvas.drawCircle(xcenter, ycenter, 5, paint);
        
        invalidate();
	}
	
	private void drawAngle(Canvas c, float angle) {
        int xcenter = c.getWidth()/2;
        int ycenter = c.getHeight()/2;
        float angleDeg = angle*180/(float)Math.PI;
        c.rotate(angleDeg, xcenter, ycenter);
        c.drawRect(xcenter-linelength/2, ycenter-linewidth/2, xcenter+linelength/2, ycenter+linewidth/2, paint);
        c.rotate(-angleDeg, xcenter, ycenter);
	}
	
	public void updateDraw(float aacc, float agyr, float akal) {
		angleAcc = aacc;
		angleGyro = agyr;
		angleKal = akal;
	}
}