package com.makis.tankcontrol;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * MapRenderView helper — converts a ROS OccupancyGrid JSON message
 * into a Bitmap that can be drawn on a Canvas.
 *
 * Also tracks the robot pose and a user-set navigation goal.
 */
public class MapRenderView {

    private Bitmap mapBitmap;
    private int    mapWidth, mapHeight;
    private float  mapResolution = 0.05f;
    private float  originX, originY;

    // Robot pose in map frame
    private float robotMapX, robotMapY, robotYaw;
    private boolean hasPose = false;

    // Goal in map frame
    private float goalMapX, goalMapY;
    private boolean hasGoal = false;

    private final Paint robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint goalPaint  = new Paint(Paint.ANTI_ALIAS_FLAG);

    public MapRenderView() {
        robotPaint.setColor(Color.RED);
        goalPaint .setColor(Color.GREEN);
        goalPaint .setStyle(Paint.Style.STROKE);
        goalPaint .setStrokeWidth(3f);
    }

    /** Parse an OccupancyGrid JSON message and build the Bitmap. */
    public synchronized void updateMap(JSONObject msg) {
        try {
            JSONObject info = msg.getJSONObject("info");
            mapWidth      = info.getInt("width");
            mapHeight     = info.getInt("height");
            mapResolution = (float) info.getDouble("resolution");
            JSONObject origin = info.getJSONObject("origin").getJSONObject("position");
            originX = (float) origin.getDouble("x");
            originY = (float) origin.getDouble("y");

            JSONArray data = msg.getJSONArray("data");
            int[] pixels = new int[mapWidth * mapHeight];

            for (int i = 0; i < data.length(); i++) {
                int val = data.getInt(i);
                int color;
                if (val < 0)        color = Color.GRAY;           // unknown
                else if (val == 0)  color = Color.WHITE;          // free
                else                color = Color.BLACK;           // occupied
                // Flip Y (ROS map origin is bottom-left)
                int row = mapHeight - 1 - (i / mapWidth);
                int col = i % mapWidth;
                pixels[row * mapWidth + col] = color;
            }
            mapBitmap = Bitmap.createBitmap(pixels, mapWidth, mapHeight,
                                            Bitmap.Config.ARGB_8888);
        } catch (Exception ignored) {}
    }

    /** Update robot pose (map frame coordinates, yaw in radians). */
    public synchronized void updatePose(float x, float y, float yaw) {
        robotMapX = x; robotMapY = y; robotYaw = yaw;
        hasPose = true;
    }

    public void setGoal(float x, float y) {
        goalMapX = x; goalMapY = y; hasGoal = true;
    }

    public boolean hasMap() { return mapBitmap != null; }

    /**
     * Draw the map, robot, and goal into the given canvas scaled to fit viewRect.
     */
    public synchronized void draw(Canvas canvas, RectF viewRect) {
        if (mapBitmap == null) return;

        float scaleX = viewRect.width()  / mapWidth;
        float scaleY = viewRect.height() / mapHeight;
        float scale  = Math.min(scaleX, scaleY);

        float drawW = mapWidth  * scale;
        float drawH = mapHeight * scale;
        float offX  = viewRect.left + (viewRect.width()  - drawW) / 2f;
        float offY  = viewRect.top  + (viewRect.height() - drawH) / 2f;

        // Draw map
        RectF dest = new RectF(offX, offY, offX + drawW, offY + drawH);
        canvas.drawBitmap(mapBitmap, null, dest, null);

        // Helper: world → screen
        // mapBitmap is flipped: row = (mapHeight-1) - worldY_in_cells
        // worldX_in_cells = (world_x - originX) / resolution

        if (hasPose) {
            float cx = ((robotMapX - originX) / mapResolution) * scale + offX;
            float cy = ((mapHeight - 1 - (robotMapY - originY) / mapResolution)) * scale + offY;
            float r  = 12f;
            canvas.drawCircle(cx, cy, r, robotPaint);
            // Direction arrow
            float ax = cx + (float) Math.cos(robotYaw) * r * 1.8f;
            float ay = cy - (float) Math.sin(robotYaw) * r * 1.8f;
            Paint arrow = new Paint(robotPaint);
            arrow.setStrokeWidth(3f);
            canvas.drawLine(cx, cy, ax, ay, arrow);
        }

        if (hasGoal) {
            float gx = ((goalMapX - originX) / mapResolution) * scale + offX;
            float gy = ((mapHeight - 1 - (goalMapY - originY) / mapResolution)) * scale + offY;
            canvas.drawCircle(gx, gy, 16f, goalPaint);
            canvas.drawLine(gx - 16, gy, gx + 16, gy, goalPaint);
            canvas.drawLine(gx, gy - 16, gx, gy + 16, goalPaint);
        }
    }

    /**
     * Convert a screen touch (within viewRect) back to map-frame world coords.
     * Returns null if no map is available.
     */
    public synchronized float[] screenToWorld(float sx, float sy, RectF viewRect) {
        if (mapBitmap == null) return null;

        float scaleX = viewRect.width()  / mapWidth;
        float scaleY = viewRect.height() / mapHeight;
        float scale  = Math.min(scaleX, scaleY);

        float drawW = mapWidth  * scale;
        float drawH = mapHeight * scale;
        float offX  = viewRect.left + (viewRect.width()  - drawW) / 2f;
        float offY  = viewRect.top  + (viewRect.height() - drawH) / 2f;

        float cellX = (sx - offX) / scale;
        float cellY = (sy - offY) / scale;

        float worldX = cellX * mapResolution + originX;
        float worldY = (mapHeight - 1 - cellY) * mapResolution + originY;
        return new float[]{worldX, worldY};
    }
}
