// TalkyMaster - App Inventor extension
// Master station for the TalkyWalky Tab5 network: discovers the Tab5 devices
// (UDP beacons), records voice (16 kHz PCM), encodes IMA-ADPCM and sends it
// over TCP with the MASTER level (QoS 3, DSCP CS7) to one or all devices.
package fr.invethic.talkymaster;

import android.Manifest;
import android.content.Context;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.net.wifi.WifiManager;

import com.google.appinventor.components.annotations.DesignerComponent;
import com.google.appinventor.components.annotations.DesignerProperty;
import com.google.appinventor.components.annotations.PropertyCategory;
import com.google.appinventor.components.annotations.SimpleEvent;
import com.google.appinventor.components.annotations.SimpleFunction;
import com.google.appinventor.components.annotations.SimpleObject;
import com.google.appinventor.components.annotations.SimpleProperty;
import com.google.appinventor.components.annotations.UsesPermissions;
import com.google.appinventor.components.common.ComponentCategory;
import com.google.appinventor.components.common.PropertyTypeConstants;
import com.google.appinventor.components.runtime.AndroidNonvisibleComponent;
import com.google.appinventor.components.runtime.ComponentContainer;
import com.google.appinventor.components.runtime.EventDispatcher;
import com.google.appinventor.components.runtime.OnDestroyListener;
import com.google.appinventor.components.runtime.PermissionResultHandler;
import com.google.appinventor.components.runtime.util.YailList;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@DesignerComponent(version = 1,
    description = "TalkyWalky MASTER station for M5Stack Tab5 devices: discovers Tab5 on the Wi-Fi network, "
        + "records push-to-talk voice and sends it (IMA-ADPCM 16 kHz, MASTER priority) to one or all devices.",
    category = ComponentCategory.EXTENSION,
    nonVisible = true,
    iconName = "images/extension.png")
@SimpleObject(external = true)
@UsesPermissions(permissionNames = "android.permission.INTERNET, android.permission.RECORD_AUDIO, "
    + "android.permission.ACCESS_WIFI_STATE, android.permission.ACCESS_NETWORK_STATE, "
    + "android.permission.CHANGE_WIFI_MULTICAST_STATE")
public class TalkyMaster extends AndroidNonvisibleComponent implements OnDestroyListener {

  private static final int UDP_PORT = 45000;
  private static final int TCP_PORT = 45001;
  private static final int SAMPLE_RATE = 16000;
  private static final int ENV_BARS = 24;
  private static final int PEER_TIMEOUT_MS = 7000;
  private static final int QOS_MASTER = 3;
  private static final int TOS_MASTER = 0xE0;          // CS7 -> WMM AC_VO
  private static final String[] LEVELS = {"LOW", "MEDIUM", "HIGH", "MASTER"};

  private static class Peer {
    String name; String ip; int level; long lastSeen;
  }

  private final Map<String, Peer> peers = new LinkedHashMap<String, Peer>();
  private volatile boolean discoveryRunning = false;
  private Thread discoveryThread;
  private DatagramSocket udpSocket;
  private WifiManager.MulticastLock multicastLock;

  private volatile boolean recording = false;
  private Thread recordThread;
  private short[] pcm;
  private volatile int pcmCount = 0;

  private short[] lastPcm;          // conditioned audio of the last message
  private byte[] lastPayload;       // ADPCM payload
  private byte[] lastEnv;
  private int lastNumSamples = 0;
  private AudioTrack track;

  private String masterName = "MASTER";
  private String masterSecret = "invethic-master";
  private int maxSeconds = 30;

  public TalkyMaster(ComponentContainer container) {
    super(container.$form());
    form.registerForOnDestroy(this);
  }

  // ============================== PROPERTIES ===============================
  @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "MASTER")
  @SimpleProperty
  public void MasterName(String name) { masterName = (name == null || name.isEmpty()) ? "MASTER" : name; }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR, description = "Name shown on the Tab5 screens.")
  public String MasterName() { return masterName; }

  @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "invethic-master")
  @SimpleProperty
  public void MasterSecret(String secret) { masterSecret = secret == null ? "" : secret; }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR,
      description = "Shared secret, must match MASTER_SECRET in the Tab5 sketch.")
  public String MasterSecret() { return masterSecret; }

  @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_NON_NEGATIVE_INTEGER, defaultValue = "30")
  @SimpleProperty
  public void MaxSeconds(int s) { maxSeconds = Math.max(1, Math.min(30, s)); }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR, description = "Maximum message length (1..30 s).")
  public int MaxSeconds() { return maxSeconds; }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR, description = "Number of Tab5 currently online.")
  public int OnlineCount() { synchronized (peers) { return peers.size(); } }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR, description = "True while recording.")
  public boolean IsRecording() { return recording; }

  @SimpleProperty(category = PropertyCategory.BEHAVIOR, description = "Duration of the last recorded message (s).")
  public double LastDuration() { return lastNumSamples / (double) SAMPLE_RATE; }

  // =============================== DISCOVERY ===============================
  @SimpleFunction(description = "Start listening for Tab5 beacons on the Wi-Fi network.")
  public void StartDiscovery() {
    if (discoveryRunning) return;
    discoveryRunning = true;
    try {
      WifiManager wm = (WifiManager) form.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
      multicastLock = wm.createMulticastLock("talkymaster");
      multicastLock.setReferenceCounted(false);
      multicastLock.acquire();
    } catch (Exception e) {
      postError("Multicast lock: " + e.getMessage());
    }
    discoveryThread = new Thread(new Runnable() {
      public void run() {
        try {
          udpSocket = new DatagramSocket(null);
          udpSocket.setReuseAddress(true);
          udpSocket.setBroadcast(true);
          udpSocket.bind(new InetSocketAddress(UDP_PORT));
          udpSocket.setSoTimeout(1000);
          byte[] buf = new byte[128];
          while (discoveryRunning) {
            try {
              DatagramPacket p = new DatagramPacket(buf, buf.length);
              udpSocket.receive(p);
              handleBeacon(new String(p.getData(), 0, p.getLength(), "UTF-8"),
                  p.getAddress().getHostAddress());
            } catch (SocketTimeoutException ignored) {
            }
            expirePeers();
          }
        } catch (Exception e) {
          if (discoveryRunning) postError("Discovery: " + e.getMessage());
        } finally {
          if (udpSocket != null) udpSocket.close();
        }
      }
    });
    discoveryThread.start();
  }

  @SimpleFunction(description = "Stop listening for Tab5 beacons.")
  public void StopDiscovery() {
    discoveryRunning = false;
    if (udpSocket != null) udpSocket.close();
    if (multicastLock != null && multicastLock.isHeld()) multicastLock.release();
  }

  private void handleBeacon(String msg, String ip) {
    if (!msg.startsWith("TWK1;")) return;
    String[] parts = msg.split(";");
    if (parts.length < 3) return;
    int level;
    try { level = Math.max(0, Math.min(2, Integer.parseInt(parts[2].trim()))); }
    catch (NumberFormatException e) { return; }
    boolean isNew = false, changed = false;
    synchronized (peers) {
      Peer p = peers.get(ip);
      if (p == null) { p = new Peer(); p.ip = ip; peers.put(ip, p); isNew = true; }
      if (!parts[1].equals(p.name) || p.level != level) changed = true;
      p.name = parts[1];
      p.level = level;
      p.lastSeen = System.currentTimeMillis();
    }
    if (isNew) {
      final String n = parts[1], i = ip, l = LEVELS[level];
      post(new Runnable() { public void run() { PeerFound(n, i, l); } });
    }
    if (isNew || changed) firePeersChanged();
  }

  private void expirePeers() {
    long now = System.currentTimeMillis();
    List<Peer> lost = new ArrayList<Peer>();
    synchronized (peers) {
      for (Peer p : new ArrayList<Peer>(peers.values())) {
        if (now - p.lastSeen > PEER_TIMEOUT_MS) { peers.remove(p.ip); lost.add(p); }
      }
    }
    for (final Peer p : lost) post(new Runnable() { public void run() { PeerLost(p.name, p.ip); } });
    if (!lost.isEmpty()) firePeersChanged();
  }

  private void firePeersChanged() {
    final int n = OnlineCount();
    post(new Runnable() { public void run() { PeersChanged(n); } });
  }

  @SimpleFunction(description = "Labels of the online Tab5, e.g. 'TAB5-3F2A  |  MEDIUM  |  192.168.1.20'.")
  public YailList GetPeerLabels() {
    List<String> out = new ArrayList<String>();
    synchronized (peers) {
      for (Peer p : peers.values()) out.add(p.name + "  |  " + LEVELS[p.level] + "  |  " + p.ip);
    }
    return YailList.makeList(out);
  }

  @SimpleFunction(description = "IP addresses of the online Tab5 (same order as GetPeerLabels).")
  public YailList GetPeerIPs() {
    List<String> out = new ArrayList<String>();
    synchronized (peers) { for (Peer p : peers.values()) out.add(p.ip); }
    return YailList.makeList(out);
  }

  @SimpleFunction(description = "Names of the online Tab5 (same order as GetPeerLabels).")
  public YailList GetPeerNames() {
    List<String> out = new ArrayList<String>();
    synchronized (peers) { for (Peer p : peers.values()) out.add(p.name); }
    return YailList.makeList(out);
  }

  @SimpleFunction(description = "QoS levels of the online Tab5 (same order as GetPeerLabels).")
  public YailList GetPeerLevels() {
    List<String> out = new ArrayList<String>();
    synchronized (peers) { for (Peer p : peers.values()) out.add(LEVELS[p.level]); }
    return YailList.makeList(out);
  }

  // =============================== RECORDING ===============================
  @SimpleFunction(description = "Start recording from the microphone (asks permission the first time).")
  public void StartRecording() {
    if (recording) return;
    if (form.isDeniedPermission(Manifest.permission.RECORD_AUDIO)) {
      form.askPermission(Manifest.permission.RECORD_AUDIO, new PermissionResultHandler() {
        public void HandlePermissionResponse(String permission, boolean granted) {
          if (!granted) ErrorOccurred("Microphone permission refused");
          else ErrorOccurred("Microphone allowed - hold the button again to talk");
        }
      });
      return;
    }
    pcm = new short[SAMPLE_RATE * maxSeconds];
    pcmCount = 0;
    recording = true;
    recordThread = new Thread(new Runnable() {
      public void run() {
        AudioRecord rec = null;
        try {
          int minBuf = AudioRecord.getMinBufferSize(SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO,
              AudioFormat.ENCODING_PCM_16BIT);
          rec = new AudioRecord(MediaRecorder.AudioSource.MIC, SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO,
              AudioFormat.ENCODING_PCM_16BIT, Math.max(minBuf, SAMPLE_RATE));
          rec.startRecording();
          short[] chunk = new short[SAMPLE_RATE / 20];   // 50 ms
          long lastPost = 0;
          while (recording && pcmCount < pcm.length) {
            int n = rec.read(chunk, 0, Math.min(chunk.length, pcm.length - pcmCount));
            if (n <= 0) continue;
            System.arraycopy(chunk, 0, pcm, pcmCount, n);
            pcmCount += n;
            long now = System.currentTimeMillis();
            if (now - lastPost > 100) {
              lastPost = now;
              int peak = 0;
              for (int i = 0; i < n; i++) peak = Math.max(peak, Math.abs((int) chunk[i]));
              final int pct = (int) Math.min(100, Math.sqrt(peak / 32768.0) * 110);
              final double secs = Math.round(pcmCount * 10.0 / SAMPLE_RATE) / 10.0;
              post(new Runnable() { public void run() { RecordingLevel(pct, secs); } });
            }
          }
          if (recording && pcmCount >= pcm.length) {
            post(new Runnable() { public void run() { MaxLengthReached(); } });
          }
        } catch (Exception e) {
          postError("Recording: " + e.getMessage());
        } finally {
          if (rec != null) { try { rec.stop(); } catch (Exception ignored) { } rec.release(); }
        }
      }
    });
    recordThread.start();
  }

  @SimpleFunction(description = "Stop recording and prepare the message. Returns its duration in seconds "
      + "(0 if too short).")
  public double StopRecording() {
    if (!recording && recordThread == null) return 0;
    recording = false;
    try { if (recordThread != null) recordThread.join(1500); } catch (InterruptedException ignored) { }
    recordThread = null;
    int n = pcmCount;
    if (n < SAMPLE_RATE / 3) {
      lastNumSamples = 0;
      ErrorOccurred("Message too short - hold the button while talking");
      return 0;
    }
    short[] s = Arrays.copyOf(pcm, n);
    conditionAudio(s);
    lastPcm = s;
    lastNumSamples = n;
    lastEnv = envelope(s);
    lastPayload = adpcmEncode(s);
    double secs = Math.round(n * 10.0 / SAMPLE_RATE) / 10.0;
    RecordingStopped(secs);
    return secs;
  }

  @SimpleFunction(description = "Play back the last recorded message on the phone.")
  public void PlayLast() {
    if (lastPcm == null) { ErrorOccurred("Nothing recorded yet"); return; }
    try {
      if (track != null) { track.stop(); track.release(); }
      track = new AudioTrack(AudioManager.STREAM_MUSIC, SAMPLE_RATE, AudioFormat.CHANNEL_OUT_MONO,
          AudioFormat.ENCODING_PCM_16BIT, lastPcm.length * 2, AudioTrack.MODE_STATIC);
      track.write(lastPcm, 0, lastPcm.length);
      track.play();
    } catch (Exception e) {
      ErrorOccurred("Playback: " + e.getMessage());
    }
  }

  // ================================ SENDING ================================
  @SimpleFunction(description = "Send the last recorded message to every online Tab5 (broadcast).")
  public void SendToAll() {
    final List<Peer> targets = new ArrayList<Peer>();
    synchronized (peers) { targets.addAll(peers.values()); }
    if (targets.isEmpty()) { ErrorOccurred("No Tab5 online"); return; }
    sendAsync(targets, true);
  }

  @SimpleFunction(description = "Send the last recorded message to one Tab5, given its IP address.")
  public void SendTo(String ip) {
    Peer t = new Peer();
    t.ip = ip;
    t.name = ip;
    synchronized (peers) { Peer p = peers.get(ip); if (p != null) t.name = p.name; }
    List<Peer> one = new ArrayList<Peer>();
    one.add(t);
    sendAsync(one, false);
  }

  private void sendAsync(final List<Peer> targets, final boolean broadcast) {
    if (lastPayload == null || lastNumSamples == 0) { ErrorOccurred("Nothing to send"); return; }
    final byte[] payload = lastPayload;
    final byte[] env = lastEnv;
    final int numSamples = lastNumSamples;
    new Thread(new Runnable() {
      public void run() {
        int ok = 0;
        for (final Peer p : targets) {
          final boolean r = sendVoice(p.ip, broadcast, payload, env, numSamples);
          if (r) ok++;
          post(new Runnable() { public void run() { SendResult(p.name, p.ip, r); } });
        }
        final int delivered = ok, total = targets.size();
        post(new Runnable() { public void run() { SendFinished(delivered, total); } });
      }
    }).start();
  }

  private boolean sendVoice(String ip, boolean broadcast, byte[] payload, byte[] env, int numSamples) {
    Socket s = new Socket();
    try {
      s.setTrafficClass(TOS_MASTER);
      s.setTcpNoDelay(true);
      s.connect(new InetSocketAddress(ip, TCP_PORT), 2000);
      s.setSoTimeout(4000);
      ByteBuffer h = ByteBuffer.allocate(72).order(ByteOrder.LITTLE_ENDIAN);
      h.put(new byte[]{'T', 'W', 'K', 'V'});
      h.put((byte) 2);                          // protocol version
      h.put((byte) QOS_MASTER);
      h.put((byte) 1);                          // codec IMA-ADPCM
      h.put((byte) (broadcast ? 1 : 0));        // flags
      h.putInt(SAMPLE_RATE);
      h.putInt(numSamples);
      h.putInt(payload.length);
      byte[] nm = new byte[24];
      byte[] src = masterName.getBytes("UTF-8");
      System.arraycopy(src, 0, nm, 0, Math.min(23, src.length));
      h.put(nm);
      h.put(env);
      h.putInt(masterAuth(numSamples, payload.length));
      OutputStream os = s.getOutputStream();
      os.write(h.array());
      os.write(payload);
      os.flush();
      InputStream is = s.getInputStream();
      int ack = is.read();
      return ack == 'A';
    } catch (Exception e) {
      return false;
    } finally {
      try { s.close(); } catch (Exception ignored) { }
    }
  }

  // FNV-1a 32 bit over secret + numSamples + payloadLen (identical on the Tab5)
  private int masterAuth(int numSamples, int payloadLen) {
    int h = 0x811C9DC5;
    byte[] sec;
    try { sec = masterSecret.getBytes("UTF-8"); } catch (Exception e) { sec = new byte[0]; }
    for (byte b : sec) { h ^= (b & 0xFF); h *= 16777619; }
    for (int i = 0; i < 4; i++) { h ^= (numSamples >>> (8 * i)) & 0xFF; h *= 16777619; }
    for (int i = 0; i < 4; i++) { h ^= (payloadLen >>> (8 * i)) & 0xFF; h *= 16777619; }
    return h;
  }

  // ================================= AUDIO =================================
  private static short clamp16(int v) { return (short) (v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); }

  private static void conditionAudio(short[] s) {
    float px = 0, py = 0;
    for (int i = 0; i < s.length; i++) {
      float x = s[i];
      float y = x - px + 0.995f * py;
      px = x; py = y;
      s[i] = clamp16((int) y);
    }
    int fade = Math.min(s.length, SAMPLE_RATE / 50);
    for (int i = 0; i < fade; i++) s[i] = (short) (s[i] * i / fade);
    int peak = 1;
    for (short v : s) peak = Math.max(peak, Math.abs((int) v));
    float g = Math.min(8.0f, 26000.0f / peak);
    for (int i = 0; i < s.length; i++) s[i] = clamp16((int) (s[i] * g));
  }

  private static byte[] envelope(short[] s) {
    byte[] env = new byte[ENV_BARS];
    int seg = s.length / ENV_BARS;
    if (seg == 0) return env;
    for (int b = 0; b < ENV_BARS; b++) {
      int peak = 0;
      for (int i = b * seg; i < (b + 1) * seg; i++) peak = Math.max(peak, Math.abs((int) s[i]));
      env[b] = (byte) (int) (Math.sqrt(peak / 32768.0) * 255.0);
    }
    return env;
  }

  private static final int[] STEP = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,
    107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,
    724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,
    3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
    15289,16818,18500,20350,22385,24623,27086,29794,32767 };
  private static final int[] INDEX = { -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8 };

  private static byte[] adpcmEncode(short[] in) {
    byte[] out = new byte[(in.length + 1) / 2];
    int pred = 0, idx = 0;
    for (int i = 0; i < in.length; i++) {
      int step = STEP[idx];
      int diff = in[i] - pred;
      int code = 0;
      if (diff < 0) { code = 8; diff = -diff; }
      int vp = step >> 3;
      if (diff >= step) { code |= 4; diff -= step; vp += step; }
      step >>= 1;
      if (diff >= step) { code |= 2; diff -= step; vp += step; }
      step >>= 1;
      if (diff >= step) { code |= 1; vp += step; }
      pred += ((code & 8) != 0) ? -vp : vp;
      pred = clamp16(pred);
      idx += INDEX[code];
      idx = idx < 0 ? 0 : (idx > 88 ? 88 : idx);
      if ((i & 1) != 0) out[i >> 1] |= (byte) (code << 4);
      else out[i >> 1] = (byte) code;
    }
    return out;
  }

  // ================================ EVENTS =================================
  @SimpleEvent(description = "A new Tab5 appeared on the network.")
  public void PeerFound(String name, String ip, String level) {
    EventDispatcher.dispatchEvent(this, "PeerFound", name, ip, level);
  }

  @SimpleEvent(description = "A Tab5 stopped sending beacons.")
  public void PeerLost(String name, String ip) {
    EventDispatcher.dispatchEvent(this, "PeerLost", name, ip);
  }

  @SimpleEvent(description = "The list of online Tab5 changed - refresh your ListView.")
  public void PeersChanged(int count) {
    EventDispatcher.dispatchEvent(this, "PeersChanged", count);
  }

  @SimpleEvent(description = "Microphone level (0..100) and elapsed time while recording.")
  public void RecordingLevel(int percent, double seconds) {
    EventDispatcher.dispatchEvent(this, "RecordingLevel", percent, seconds);
  }

  @SimpleEvent(description = "Recording stopped, message ready to send.")
  public void RecordingStopped(double seconds) {
    EventDispatcher.dispatchEvent(this, "RecordingStopped", seconds);
  }

  @SimpleEvent(description = "MaxSeconds reached - release the button to send.")
  public void MaxLengthReached() {
    EventDispatcher.dispatchEvent(this, "MaxLengthReached");
  }

  @SimpleEvent(description = "Result of the delivery to one Tab5.")
  public void SendResult(String name, String ip, boolean success) {
    EventDispatcher.dispatchEvent(this, "SendResult", name, ip, success);
  }

  @SimpleEvent(description = "All deliveries finished.")
  public void SendFinished(int delivered, int total) {
    EventDispatcher.dispatchEvent(this, "SendFinished", delivered, total);
  }

  @SimpleEvent(description = "An error or information message.")
  public void ErrorOccurred(String message) {
    EventDispatcher.dispatchEvent(this, "ErrorOccurred", message);
  }

  // ================================ HELPERS ================================
  private void post(Runnable r) { form.runOnUiThread(r); }

  private void postError(final String m) {
    post(new Runnable() { public void run() { ErrorOccurred(m); } });
  }

  @Override
  public void onDestroy() {
    StopDiscovery();
    recording = false;
    if (track != null) { try { track.release(); } catch (Exception ignored) { } }
  }
}
