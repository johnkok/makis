package com.makis.tankcontrol;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

/**
 * JoystickView — circular joystick widget.
 *
 * Returns normalised [-1, +1] axes via JoystickListener.
 *   X-axis: right = +1
 *   Y-axis: up    = +1   (inverted screen Y)
 */
public class JoystickView extends View {

    public interface JoystickListener {
        void onJoystickMoved(float x, float y);
    }

    private final Paint bgPaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint stickPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint rimPaint   = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float centreX, centreY, baseRadius, stickRadius;
    private float stickX, stickY;

    private JoystickListener listener;

    public JoystickView(Context ctx) { this(ctx, null); }
    public JoystickView(Context ctx, AttributeSet attrs) {
        super(ctx, attrs);
        bgPaint   .setColor(Color.argb(80, 200, 200, 200));
        rimPaint  .setColor(Color.argb(160, 100, 100, 100));
        rimPaint  .setStyle(Paint.Style.STROKE);
        rimPaint  .setStrokeWidth(4f);
        stickPaint.setColor(Color.argb(200, 50, 150, 255));
    }

    @Override
    protected void onSizeChanged(int w, int h, int ow, int oh) {
        centreX    = w / 2f;
        centreY    = h / 2f;
        baseRadius = Math.min(w, h) / 2f - 8f;
        stickRadius = baseRadius * 0.35f;
        stickX = centreX;
        stickY = centreY;
    }

    @Override
    protected void onDraw(Canvas c) {
        c.drawCircle(centreX, centreY, baseRadius, bgPaint);
        c.drawCircle(centreX, centreY, baseRadius, rimPaint);
        c.drawCircle(stickX,  stickY,  stickRadius, stickPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        float dx = e.getX() - centreX;
        float dy = e.getY() - centreY;
        float dist = (float) Math.sqrt(dx * dx + dy * dy);

        if (e.getAction() == MotionEvent.ACTION_UP ||
            e.getAction() == MotionEvent.ACTION_CANCEL) {
            stickX = centreX;
            stickY = centreY;
            if (listener != null) listener.onJoystickMoved(0f, 0f);
            invalidate();
            return true;
        }

        float limit = baseRadius - stickRadius;
        if (dist > limit) {
            dx = dx / dist * limit;
            dy = dy / dist * limit;
        }
        stickX = centreX + dx;
        stickY = centreY + dy;

        float normX =  dx / limit;
        float normY = -dy / limit;   // invert Y
        if (listener != null) listener.onJoystickMoved(normX, normY);
        invalidate();
        return true;
    }

    public void setJoystickListener(JoystickListener l) { listener = l; }
}
