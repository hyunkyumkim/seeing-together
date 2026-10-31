package com.example.imustream

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.media.ImageReader
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.WindowManager
import android.widget.TextView
import java.io.File

/**
 * Camera access feasibility test (no preview, no extra dependencies).
 * Lists cameras, opens one, captures 3 JPEG stills, saves them to the app's external files dir,
 * and logs everything under the tag CAMTEST.
 *
 * Run:  adb shell pm grant com.example.imustream android.permission.CAMERA
 *       adb shell am start -n com.example.imustream/.CameraTestActivity --es id 0
 * Read: adb logcat -d -s CAMTEST
 */
class CameraTestActivity : Activity() {
    private val logTag = "CAMTEST"
    private lateinit var tv: TextView
    private val bgThread = HandlerThread("cam-bg").apply { start() }
    private lateinit var bg: Handler
    private var camera: CameraDevice? = null
    private var reader: ImageReader? = null
    private val lines = ArrayList<String>()

    private fun log(msg: String) {
        Log.i(logTag, msg)
        synchronized(lines) { lines.add(msg) }
        runOnUiThread { tv.text = synchronized(lines) { lines.takeLast(12).joinToString("\n") } }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        tv = TextView(this).apply { textSize = 14f }
        setContentView(tv)
        bg = Handler(bgThread.looper)

        val cm = getSystemService(CAMERA_SERVICE) as CameraManager
        val ids = try {
            cm.cameraIdList
        } catch (e: Exception) {
            log("cameraIdList error: $e"); return
        }
        log("camera ids: ${ids.joinToString()}")
        for (id in ids) {
            val c = cm.getCameraCharacteristics(id)
            val facing = c.get(CameraCharacteristics.LENS_FACING)
            val sizes = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                ?.getOutputSizes(ImageFormat.JPEG)
            log("id=$id facing=$facing jpegSizes(first3)=${sizes?.take(3)?.joinToString()}")
        }
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            log("CAMERA permission NOT granted (run adb shell pm grant ...)")
            return
        }
        val id = intent.getStringExtra("id") ?: ids.firstOrNull()
        if (id == null) {
            log("no camera found"); return
        }
        openAndCapture(cm, id)
    }

    private fun openAndCapture(cm: CameraManager, id: String) {
        val map = cm.getCameraCharacteristics(id)
            .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        if (map == null) {
            log("no stream configuration map"); return
        }
        val all = map.getOutputSizes(ImageFormat.JPEG)
        val size = all.filter { it.width * it.height <= 1920 * 1080 }
            .maxByOrNull { it.width * it.height } ?: all.first()
        log("using id=$id size=${size.width}x${size.height}")

        val r = ImageReader.newInstance(size.width, size.height, ImageFormat.JPEG, 2)
        reader = r
        r.setOnImageAvailableListener({ rd ->
            val img = rd.acquireLatestImage() ?: return@setOnImageAvailableListener
            val buf = img.planes[0].buffer
            val bytes = ByteArray(buf.remaining())
            buf.get(bytes)
            img.close()
            val f = File(getExternalFilesDir(null), "cam${id}_${System.currentTimeMillis()}.jpg")
            f.writeBytes(bytes)
            log("saved ${f.absolutePath} (${bytes.size} bytes)")
        }, bg)

        try {
            cm.openCamera(id, object : CameraDevice.StateCallback() {
                override fun onOpened(cam: CameraDevice) {
                    camera = cam
                    log("camera opened")
                    cam.createCaptureSession(
                        listOf(r.surface),
                        object : CameraCaptureSession.StateCallback() {
                            override fun onConfigured(s: CameraCaptureSession) {
                                log("session configured; 3 captures starting in 2 s")
                                repeat(3) { i ->
                                    bg.postDelayed({
                                        try {
                                            val req = cam.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE)
                                            req.addTarget(r.surface)
                                            s.capture(req.build(), null, bg)
                                        } catch (e: Exception) {
                                            log("capture error: $e")
                                        }
                                    }, 2000L + i * 1000L)
                                }
                            }

                            override fun onConfigureFailed(s: CameraCaptureSession) {
                                log("session configure FAILED")
                            }
                        },
                        bg
                    )
                }

                override fun onDisconnected(cam: CameraDevice) {
                    log("camera disconnected"); cam.close()
                }

                override fun onError(cam: CameraDevice, error: Int) {
                    log("camera error code=$error"); cam.close()
                }
            }, bg)
        } catch (e: Exception) {
            log("openCamera exception: $e")
        }
    }

    override fun onDestroy() {
        camera?.close()
        reader?.close()
        bgThread.quitSafely()
        super.onDestroy()
    }
}
