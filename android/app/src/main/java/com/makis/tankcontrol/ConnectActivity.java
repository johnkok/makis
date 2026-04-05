package com.makis.tankcontrol;

import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

/**
 * ConnectActivity — splash screen where user enters RPi 5 IP address.
 */
public class ConnectActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connect);

        EditText ipInput  = findViewById(R.id.editTextIp);
        EditText portInput = findViewById(R.id.editTextPort);
        Button   connectBtn = findViewById(R.id.buttonConnect);

        // Restore last used values
        String lastIp   = getPreferences(MODE_PRIVATE).getString("ip",   "192.168.1.100");
        String lastPort = getPreferences(MODE_PRIVATE).getString("port",  "9090");
        ipInput.setText(lastIp);
        portInput.setText(lastPort);

        connectBtn.setOnClickListener(v -> {
            String ip   = ipInput.getText().toString().trim();
            String port = portInput.getText().toString().trim();
            if (ip.isEmpty()) {
                Toast.makeText(this, "Enter RPi 5 IP address", Toast.LENGTH_SHORT).show();
                return;
            }
            getPreferences(MODE_PRIVATE).edit()
                    .putString("ip", ip).putString("port", port).apply();

            Intent intent = new Intent(this, MainActivity.class);
            intent.putExtra("ip", ip);
            intent.putExtra("port", Integer.parseInt(port.isEmpty() ? "9090" : port));
            startActivity(intent);
        });
    }
}
