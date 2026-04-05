package com.makis.tankcontrol;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * MapFragment — displays the live SLAM occupancy grid and allows
 * tapping to set a navigation goal.
 */
public class MapFragment extends Fragment implements SurfaceHolder.Callback {

    private SurfaceView surface;
    private SurfaceHolder holder;
    private MapRenderView renderer;
    private RosBridgeClient ros;
    private TextView statusTv;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_map, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View v, @Nullable Bundle s) {
        ros      = ((MainActivity) requireActivity()).getRosClient();
        surface  = v.findViewById(R.id.mapSurface);
        statusTv = v.findViewById(R.id.textMapStatus);
        renderer = new MapRenderView();

        holder = surface.getHolder();
        holder.addCallback(this);

        // Subscribe to map
        ros.subscribe("/map", "nav_msgs/OccupancyGrid", msg -> {
            renderer.updateMap(msg);
            redraw();
            requireActivity().runOnUiThread(() ->
                    statusTv.setText("Map received"));
        });

        // Subscribe to robot pose (from EKF)
        ros.subscribe("/odometry/filtered", "nav_msgs/Odometry", msg -> {
            try {
                JSONObject pos = msg.getJSONObject("pose")
                                    .getJSONObject("pose")
                                    .getJSONObject("position");
                JSONObject ori = msg.getJSONObject("pose")
                                    .getJSONObject("pose")
                                    .getJSONObject("orientation");
                float x = (float) pos.getDouble("x");
                float y = (float) pos.getDouble("y");
                // Get yaw from quaternion
                float qz = (float) ori.getDouble("z");
                float qw = (float) ori.getDouble("w");
                float yaw = 2f * (float) Math.atan2(qz, qw);
                renderer.updatePose(x, y, yaw);
                redraw();
            } catch (Exception ignored) {}
        });

        // Tap to set goal
        surface.setOnTouchListener((sv, e) -> {
            if (e.getAction() == MotionEvent.ACTION_UP) {
                RectF rect = new RectF(0, 0, sv.getWidth(), sv.getHeight());
                float[] world = renderer.screenToWorld(e.getX(), e.getY(), rect);
                if (world != null) {
                    renderer.setGoal(world[0], world[1]);
                    ros.sendGoal(world[0], world[1], 0.0);
                    redraw();
                    requireActivity().runOnUiThread(() ->
                            statusTv.setText(String.format("Goal: (%.2f, %.2f)",
                                    world[0], world[1])));
                } else {
                    Toast.makeText(getContext(), "No map yet", Toast.LENGTH_SHORT).show();
                }
            }
            return true;
        });
    }

    private void redraw() {
        if (holder == null || !renderer.hasMap()) return;
        Canvas c = holder.lockCanvas();
        if (c == null) return;
        try {
            c.drawColor(Color.DKGRAY);
            renderer.draw(c, new RectF(0, 0, c.getWidth(), c.getHeight()));
        } finally {
            holder.unlockCanvasAndPost(c);
        }
    }

    @Override public void surfaceCreated(@NonNull SurfaceHolder h)  { redraw(); }
    @Override public void surfaceChanged(@NonNull SurfaceHolder h, int f, int w, int ht) { redraw(); }
    @Override public void surfaceDestroyed(@NonNull SurfaceHolder h) {}

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        if (ros != null) {
            ros.unsubscribe("/map");
            ros.unsubscribe("/odometry/filtered");
        }
    }
}
