package com.makis.tankcontrol;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.ToggleButton;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

/**
 * ControlFragment — two joysticks for mecanum drive:
 *   Left  joystick → linear X (fwd/back) + linear Y (strafe)
 *   Right joystick → angular Z (rotate)
 *
 * Speed multiplier slider goes from 0.05 m/s to 0.15 m/s.
 */
public class ControlFragment extends Fragment {

    private RosBridgeClient ros;
    private float leftX, leftY, rightX;
    private static final float MAX_LINEAR  = 0.12f;  // m/s
    private static final float MAX_ANGULAR = 0.8f;   // rad/s
    private boolean publishing = false;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_control, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View v, @Nullable Bundle s) {
        ros = ((MainActivity) requireActivity()).getRosClient();
        ros.advertise("/cmd_vel", "geometry_msgs/Twist");

        JoystickView leftJoy  = v.findViewById(R.id.joystickLeft);
        JoystickView rightJoy = v.findViewById(R.id.joystickRight);
        TextView     statusTv = v.findViewById(R.id.textStatus);
        ToggleButton stopBtn  = v.findViewById(R.id.buttonStop);

        leftJoy.setJoystickListener((x, y) -> {
            leftX = x; leftY = y;
            publish(statusTv);
        });

        rightJoy.setJoystickListener((x, y) -> {
            rightX = x;
            publish(statusTv);
        });

        stopBtn.setOnCheckedChangeListener((btn, checked) -> {
            if (checked) {
                publishing = false;
                ros.sendCmdVel(0, 0, 0);
                statusTv.setText("STOPPED");
            } else {
                publishing = true;
            }
        });
        publishing = true;
    }

    private void publish(TextView status) {
        if (!publishing) return;
        float vx    =  leftY  * MAX_LINEAR;
        float vy    =  leftX  * MAX_LINEAR;
        float omega = -rightX * MAX_ANGULAR;
        ros.sendCmdVel(vx, vy, omega);
        if (status != null) {
            String txt = String.format("vx=%.2f vy=%.2f ω=%.2f", vx, vy, omega);
            status.setText(txt);
        }
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        if (ros != null) ros.sendCmdVel(0, 0, 0);
    }
}
