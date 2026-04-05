package com.makis.tankcontrol;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

/**
 * CameraFragment — displays the Pi Camera live stream via web_video_server.
 *
 * Default URL: http://<rpi5_ip>:8080/stream?topic=/camera/image_raw&type=mjpeg
 */
public class CameraFragment extends Fragment {

    private WebView webView;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_camera, container, false);
    }

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    public void onViewCreated(@NonNull View v, @Nullable Bundle s) {
        webView = v.findViewById(R.id.cameraWebView);
        WebSettings ws = webView.getSettings();
        ws.setJavaScriptEnabled(true);
        ws.setLoadWithOverviewMode(true);
        ws.setUseWideViewPort(true);
        webView.setWebViewClient(new WebViewClient());

        // Build stream URL from the IP passed to MainActivity
        String ip = requireActivity().getIntent().getStringExtra("ip");
        if (ip == null) ip = "192.168.1.100";
        String url = "http://" + ip + ":8080/stream?topic=/camera/image_raw&type=mjpeg";

        EditText urlEdit = v.findViewById(R.id.editStreamUrl);
        urlEdit.setText(url);

        Button loadBtn = v.findViewById(R.id.buttonLoadStream);
        loadBtn.setOnClickListener(btn ->
                webView.loadUrl(urlEdit.getText().toString().trim()));

        // Auto-load
        webView.loadUrl(url);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        if (webView != null) webView.destroy();
    }
}
