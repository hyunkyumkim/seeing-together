package com.example.imustream

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.Image
import android.media.ImageReader
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.util.Size
import android.view.WindowManager
import android.widget.TextView
import java.io.BufferedOutputStream
import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * Camera frame streaming test: Camera2 (YUV) -> JPEG -> TCP to the laptop (stream_receiver.py).
 *
 * Every camera frame gets a sequence number. Frames are dropped on the glasses when the encoder or
 * the network is busy, so gaps in seq at the receiver = frames that never left the glasses.
 *
 * Packet: "FRM1" | seq:int64 | captureTsNs:int64 | encodedNs:int64 | devicePipeUs:int32 |
 *         width:int32 | height:int32 | jpegLen:int32 | jpeg bytes      (big-endian, 44-byte header)
 *   captureTsNs  = Image.timestamp (sensor time)
 *   encodedNs    = SystemClock.elapsedRealtimeNanos() when the JPEG was ready
 *   devicePipeUs = encodedNs - captureTsNs in us if the sensor timestamp source is REALTIME, else -1
 *
 * Wireless:  adb shell am start -n com.example.imustream/.FrameStreamActivity --es ip <laptop IP> --ei port 5006 --ei w 640 --ei h 480 --ei q 80
 * USB:       adb reverse tcp:5006 tcp:5006   then the same command with --es ip 127.0.0.1
 * The screen must stay on and this app must be in the foreground, or the camera is blocked.
 * Logs: adb logcat -s STREAMTEST
 */
class FrameStreamActivity : Activity() {
    private val tag = "STREAMTEST"
    private lateinit var tv: TextView
    private val main = Handler(Looper.getMainLooper())

    private val camThread = HandlerThread("cam-bg").apply { start() }
    private lateinit var camHandler: Handler
    private val encoder = Executors.newSingleThreadExecutor()
    private val encoderBusy = AtomicBoolean(false)
    private val sendQueue = ArrayBlockingQueue<Frame>(2)
    @Volatile private var running = true

    private var camera: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var reader: ImageReader? = null

    // parameters (adb intent extras)
    private var ip = "127.0.0.1"
    private var port = 5006
    private var reqW = 640
    private var reqH = 480
    private var quality = 80
    private var fps = 30
    private var camId = "0"
    private var realtimeTs = false

    // counters
    private val seq = AtomicLong(0)
    private val encoded = AtomicLong(0)
    private val sent = AtomicLong(0)
    private val dropBusy = AtomicLong(0)
    private val dropQueue = AtomicLong(0)
    private val reconnects = AtomicLong(0)
    @Volatile private var connected = false
    @Volatile private var lastEncodeMs = 0L
    @Volatile private var lastConvMs = 0L
    @Volatile private var lastJpegBytes = 0
    @Volatile private var size = Size(0, 0)
    @Volatile private var status = "starting"

    private class Frame(
        val seq: Long, val captureTsNs: Long, val encodedNs: Long, val devicePipeUs: Int,
        val w: Int, val h: Int, val jpeg: ByteArray
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        tv = TextView(this).apply { textSize = 14f }
        setContentView(tv)
        camHandler = Handler(camThread.looper)

        ip = intent.getStringExtra("ip") ?: ip
        port = intent.getIntExtra("port", port)
        reqW = intent.getIntExtra("w", reqW)
        reqH = intent.getIntExtra("h", reqH)
        quality = intent.getIntExtra("q", quality)
        fps = intent.getIntExtra("fps", fps)
        camId = intent.getStringExtra("id") ?: camId
        Log.i(tag, "params ip=$ip port=$port req=${reqW}x$reqH q=$quality fps=$fps id=$camId")

        Thread(::senderLoop, "tcp-sender").start()
        main.post(object : Runnable {
            override fun run() {
                tv.text = "to $ip:$port  ${size.width}x${size.height} q=$quality  $status\n" +
                    "frames=${seq.get()} enc=${encoded.get()} sent=${sent.get()}\n" +
                    "dropBusy=${dropBusy.get()} dropQueue=${dropQueue.get()} reconnects=${reconnects.get()} " +
                    "connected=$connected\nconv=${lastConvMs} ms  encode=${lastEncodeMs} ms  jpeg=${lastJpegBytes / 1024} KB"
                if (running) main.postDelayed(this, 500)
            }
        })
    }

    override fun onResume() {
        super.onResume()
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            status = "CAMERA permission NOT granted"; Log.e(tag, status); return
        }
        openCamera()
    }

    override fun onPause() {
        closeCamera()
        super.onPause()
    }

    override fun onDestroy() {
        running = false
        closeCamera()
        encoder.shutdownNow()
        camThread.quitSafely()
        super.onDestroy()
    }

    private fun openCamera() {
        val cm = getSystemService(CAMERA_SERVICE) as CameraManager
        try {
            val ch = cm.getCameraCharacteristics(camId)
            realtimeTs = ch.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
            val map = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                ?: run { status = "no stream config map"; return }
            val sizes = map.getOutputSizes(ImageFormat.YUV_420_888)
            val chosen = sizes.firstOrNull { it.width == reqW && it.height == reqH }
                ?: sizes.minByOrNull { Math.abs(it.width * it.height - reqW * reqH) }!!
            size = chosen
            val ranges = ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            val fpsRange = ranges?.firstOrNull { it.lower == fps && it.upper == fps }
                ?: ranges?.firstOrNull { it.upper == fps }
            Log.i(tag, "size=${chosen.width}x${chosen.height} realtimeTs=$realtimeTs fpsRange=$fpsRange " +
                "available=${ranges?.joinToString()}")

            val r = ImageReader.newInstance(chosen.width, chosen.height, ImageFormat.YUV_420_888, 4)
            reader = r
            r.setOnImageAvailableListener({ rd -> onImage(rd) }, camHandler)

            cm.openCamera(camId, object : CameraDevice.StateCallback() {
                override fun onOpened(cam: CameraDevice) {
                    camera = cam
                    cam.createCaptureSession(listOf(r.surface), object : CameraCaptureSession.StateCallback() {
                        override fun onConfigured(s: CameraCaptureSession) {
                            session = s
                            val req = cam.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
                            req.addTarget(r.surface)
                            if (fpsRange != null) req.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
                            s.setRepeatingRequest(req.build(), null, camHandler)
                            status = "streaming"; Log.i(tag, "repeating request started")
                        }

                        override fun onConfigureFailed(s: CameraCaptureSession) {
                            status = "session configure FAILED"; Log.e(tag, status)
                        }
                    }, camHandler)
                }

                override fun onDisconnected(cam: CameraDevice) {
                    status = "camera disconnected"; Log.e(tag, status); cam.close()
                }

                override fun onError(cam: CameraDevice, error: Int) {
                    status = "camera error $error"; Log.e(tag, status); cam.close()
                }
            }, camHandler)
        } catch (e: Exception) {
            status = "openCamera exception: $e"; Log.e(tag, status)
        }
    }

    private fun closeCamera() {
        try { session?.close() } catch (_: Exception) {}
        try { camera?.close() } catch (_: Exception) {}
        try { reader?.close() } catch (_: Exception) {}
        session = null; camera = null; reader = null
    }

    /** Runs on cam-bg for every camera frame. Copies the frame out and hands it to the encoder if idle. */
    private fun onImage(rd: ImageReader) {
        val img = rd.acquireNextImage() ?: return
        val s = seq.getAndIncrement()
        if (!encoderBusy.compareAndSet(false, true)) {
            dropBusy.incrementAndGet(); img.close(); return
        }
        val ts = img.timestamp
        val w = img.width
        val h = img.height
        val c0 = SystemClock.elapsedRealtime()
        val nv21 = try {
            toNv21(img).also { lastConvMs = SystemClock.elapsedRealtime() - c0 }
        } catch (e: Exception) {
            Log.e(tag, "YUV copy error: $e"); encoderBusy.set(false); return
        } finally {
            img.close()
        }
        if (!running) { encoderBusy.set(false); return }
        encoder.execute {
            try {
                val t0 = SystemClock.elapsedRealtime()
                val bos = ByteArrayOutputStream(w * h / 4)
                YuvImage(nv21, ImageFormat.NV21, w, h, null).compressToJpeg(Rect(0, 0, w, h), quality, bos)
                val jpeg = bos.toByteArray()
                val encNs = SystemClock.elapsedRealtimeNanos()
                lastEncodeMs = SystemClock.elapsedRealtime() - t0
                lastJpegBytes = jpeg.size
                encoded.incrementAndGet()
                val pipeUs = if (realtimeTs) ((encNs - ts) / 1000).toInt() else -1
                if (!sendQueue.offer(Frame(s, ts, encNs, pipeUs, w, h, jpeg))) dropQueue.incrementAndGet()
            } catch (e: Exception) {
                Log.e(tag, "encode error: $e")
            } finally {
                encoderBusy.set(false)
            }
        }
    }

    /** YUV_420_888 -> NV21 (Y plane, then interleaved V/U), handling row and pixel strides. */
    private fun toNv21(img: Image): ByteArray {
        val w = img.width
        val h = img.height
        val out = ByteArray(w * h * 3 / 2)
        val yP = img.planes[0]
        val uP = img.planes[1]
        val vP = img.planes[2]
        val yBuf = yP.buffer
        var pos = 0
        if (yP.pixelStride == 1) {
            for (r in 0 until h) {
                yBuf.position(r * yP.rowStride)
                yBuf.get(out, pos, w)
                pos += w
            }
        } else {
            for (r in 0 until h) for (c in 0 until w) out[pos++] = yBuf.get(r * yP.rowStride + c * yP.pixelStride)
        }
        val uBuf = uP.buffer
        val vBuf = vP.buffer
        // Read each chroma row with one bulk get, then interleave in plain arrays
        // (per-pixel ByteBuffer.get(index) is very slow on this SoC).
        val vRow = ByteArray(vP.rowStride)
        val uRow = ByteArray(uP.rowStride)
        val vps = vP.pixelStride
        val ups = uP.pixelStride
        for (r in 0 until h / 2) {
            vBuf.position(r * vP.rowStride)
            vBuf.get(vRow, 0, minOf(vP.rowStride, vBuf.remaining()))
            uBuf.position(r * uP.rowStride)
            uBuf.get(uRow, 0, minOf(uP.rowStride, uBuf.remaining()))
            for (c in 0 until w / 2) {
                out[pos++] = vRow[c * vps]
                out[pos++] = uRow[c * ups]
            }
        }
        return out
    }

    /** Connects to the laptop and sends frames; reconnects after any error. */
    private fun senderLoop() {
        while (running) {
            var sock: Socket? = null
            try {
                sock = Socket()
                sock.tcpNoDelay = true
                sock.connect(InetSocketAddress(ip, port), 2000)
                connected = true
                Log.i(tag, "connected to $ip:$port")
                val out = DataOutputStream(BufferedOutputStream(sock.getOutputStream(), 1 shl 16))
                while (running) {
                    val f = sendQueue.poll(500, TimeUnit.MILLISECONDS) ?: continue
                    out.writeBytes("FRM1")
                    out.writeLong(f.seq)
                    out.writeLong(f.captureTsNs)
                    out.writeLong(f.encodedNs)
                    out.writeInt(f.devicePipeUs)
                    out.writeInt(f.w)
                    out.writeInt(f.h)
                    out.writeInt(f.jpeg.size)
                    out.write(f.jpeg)
                    out.flush()
                    sent.incrementAndGet()
                }
            } catch (e: Exception) {
                if (connected) Log.e(tag, "send error: $e")
            } finally {
                connected = false
                try { sock?.close() } catch (_: Exception) {}
            }
            if (running) {
                reconnects.incrementAndGet()
                SystemClock.sleep(1000)
            }
        }
    }
}
