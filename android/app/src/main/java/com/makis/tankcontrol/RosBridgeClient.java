package com.makis.tankcontrol;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONException;
import org.json.JSONObject;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * RosBridgeClient — thin wrapper around rosbridge v2 WebSocket protocol.
 *
 * Usage:
 *   client.advertise("/cmd_vel", "geometry_msgs/Twist");
 *   client.publish("/cmd_vel", msgJson);
 *   client.subscribe("/map", "nav_msgs/OccupancyGrid", msg -> { ... });
 */
public class RosBridgeClient {

    public interface MessageListener { void onMessage(JSONObject msg); }
    public interface SimpleListener    { void run(); }
    public interface ErrorListener     { void onError(Exception e); }

    private WebSocketClient ws;
    private final String ip;
    private final int    port;

    private final Map<String, MessageListener> subscribers = new ConcurrentHashMap<>();
    private SimpleListener onOpen;
    private SimpleListener onClose;
    private ErrorListener  onError;

    public RosBridgeClient(String ip, int port) {
        this.ip   = ip;
        this.port = port;
    }

    // ── Connection ────────────────────────────────────────────
    public void connect() {
        URI uri;
        try {
            uri = new URI("ws://" + ip + ":" + port);
        } catch (URISyntaxException e) {
            if (onError != null) onError.onError(e);
            return;
        }

        ws = new WebSocketClient(uri) {
            @Override
            public void onOpen(ServerHandshake hs) {
                if (onOpen != null) onOpen.run();
            }

            @Override
            public void onMessage(String text) {
                try {
                    JSONObject json = new JSONObject(text);
                    String op    = json.optString("op");
                    String topic = json.optString("topic");
                    if ("publish".equals(op) && subscribers.containsKey(topic)) {
                        MessageListener cb = subscribers.get(topic);
                        if (cb != null) cb.onMessage(json.getJSONObject("msg"));
                    }
                } catch (JSONException ignored) {}
            }

            @Override
            public void onClose(int code, String reason, boolean remote) {
                if (onClose != null) onClose.run();
            }

            @Override
            public void onError(Exception e) {
                if (onError != null) onError.onError(e);
            }
        };
        ws.connect();
    }

    public void disconnect() {
        if (ws != null && !ws.isClosed()) ws.close();
    }

    public boolean isConnected() {
        return ws != null && ws.isOpen();
    }

    // ── rosbridge ops ─────────────────────────────────────────
    public void advertise(String topic, String type) {
        send(new JSONObject(Map.of("op", "advertise", "topic", topic, "type", type)));
    }

    public void unadvertise(String topic) {
        send(new JSONObject(Map.of("op", "unadvertise", "topic", topic)));
    }

    public void publish(String topic, JSONObject msg) {
        try {
            JSONObject pkt = new JSONObject();
            pkt.put("op", "publish");
            pkt.put("topic", topic);
            pkt.put("msg", msg);
            send(pkt);
        } catch (JSONException ignored) {}
    }

    public void subscribe(String topic, String type, MessageListener cb) {
        subscribers.put(topic, cb);
        try {
            JSONObject pkt = new JSONObject();
            pkt.put("op",    "subscribe");
            pkt.put("topic", topic);
            pkt.put("type",  type);
            pkt.put("throttle_rate", 200);  // ms
            send(pkt);
        } catch (JSONException ignored) {}
    }

    public void unsubscribe(String topic) {
        subscribers.remove(topic);
        send(new JSONObject(Map.of("op", "unsubscribe", "topic", topic)));
    }

    /** Publish a 2D navigation goal (PoseStamped). */
    public void sendGoal(double x, double y, double yaw) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        try {
            JSONObject pose = new JSONObject();
            pose.put("position",    new JSONObject(Map.of("x", x, "y", y, "z", 0.0)));
            pose.put("orientation", new JSONObject(Map.of(
                    "x", 0.0, "y", 0.0, "z", sy, "w", cy)));
            JSONObject header = new JSONObject(Map.of(
                    "frame_id", "map",
                    "stamp", new JSONObject(Map.of("sec", 0, "nanosec", 0))));
            JSONObject msg = new JSONObject(Map.of("header", header, "pose", pose));
            publish("/goal_pose", msg);
        } catch (JSONException ignored) {}
    }

    /** Publish Twist cmd_vel. */
    public void sendCmdVel(double vx, double vy, double omega) {
        try {
            JSONObject msg = new JSONObject();
            msg.put("linear",  new JSONObject(Map.of("x", vx, "y", vy,    "z", 0.0)));
            msg.put("angular", new JSONObject(Map.of("x", 0.0, "y", 0.0, "z", omega)));
            publish("/cmd_vel", msg);
        } catch (JSONException ignored) {}
    }

    // ── Listener setters ──────────────────────────────────────
    public void setOnOpenListener(SimpleListener l)  { onOpen  = l; }
    public void setOnCloseListener(SimpleListener l) { onClose = l; }
    public void setOnErrorListener(ErrorListener l)  { onError = l; }

    // ── Helpers ───────────────────────────────────────────────
    private void send(JSONObject obj) {
        if (isConnected()) ws.send(obj.toString());
    }
}
