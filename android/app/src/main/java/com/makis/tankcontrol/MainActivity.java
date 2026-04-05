package com.makis.tankcontrol;

import android.os.Bundle;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;
import androidx.viewpager2.adapter.FragmentStateAdapter;
import androidx.viewpager2.widget.ViewPager2;

import com.google.android.material.tabs.TabLayout;
import com.google.android.material.tabs.TabLayoutMediator;

/**
 * MainActivity — hosts three tabs:
 *   0 Control  (joystick manual drive)
 *   1 Map      (SLAM occupancy grid + goal setter)
 *   2 Camera   (live MJPEG stream)
 */
public class MainActivity extends AppCompatActivity {

    private RosBridgeClient rosClient;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        String ip   = getIntent().getStringExtra("ip");
        int    port = getIntent().getIntExtra("port", 9090);

        rosClient = new RosBridgeClient(ip, port);
        rosClient.connect();

        rosClient.setOnOpenListener(() -> runOnUiThread(() ->
                Toast.makeText(this, "Connected to " + ip, Toast.LENGTH_SHORT).show()));
        rosClient.setOnErrorListener(e -> runOnUiThread(() ->
                Toast.makeText(this, "WS error: " + e.getMessage(), Toast.LENGTH_LONG).show()));
        rosClient.setOnCloseListener(() -> runOnUiThread(() ->
                Toast.makeText(this, "Disconnected", Toast.LENGTH_SHORT).show()));

        ViewPager2 pager  = findViewById(R.id.viewPager);
        TabLayout  tabs   = findViewById(R.id.tabLayout);

        pager.setAdapter(new TabAdapter(this));
        pager.setUserInputEnabled(false); // disable swipe so joystick works properly

        new TabLayoutMediator(tabs, pager, (tab, pos) -> {
            switch (pos) {
                case 0: tab.setText("Control"); break;
                case 1: tab.setText("Map");     break;
                case 2: tab.setText("Camera");  break;
            }
        }).attach();
    }

    public RosBridgeClient getRosClient() { return rosClient; }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (rosClient != null) rosClient.disconnect();
    }

    // ── Tab adapter ──────────────────────────────────────────
    private static class TabAdapter extends FragmentStateAdapter {
        TabAdapter(FragmentActivity fa) { super(fa); }

        @Override public int getItemCount() { return 3; }

        @Override
        public Fragment createFragment(int pos) {
            switch (pos) {
                case 1: return new MapFragment();
                case 2: return new CameraFragment();
                default: return new ControlFragment();
            }
        }
    }
}
