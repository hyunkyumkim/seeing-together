package com.example.imustream

import android.app.Activity
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.view.WindowManager
import android.widget.TextView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.LinkedBlockingQueue
import kotlin.concurrent.thread

/**
 * Minimal IMU streamer: accelerometer (A) + gyroscope (G) -> UDP text lines.
 * Line format: tag,seq,timestampNs,x,y,z   (seq is per sensor; gaps = dropped samples)
 *
 * Start from the PC (no rebuild needed to change target):
 *   adb shell am start -n com.example.imustream/.MainActivity --es ip 192.168.0.10 --ei port 5005 --ei us 10000
 *   us = requested sampling period in microseconds (10000 = 100 Hz)
 */
class MainActivity : Activity(), SensorEventListener {
    private lateinit var sm: SensorManager
    private lateinit var tv: TextView
    private val queue = LinkedBlockingQueue<String>(200_000)
    @Volatile private var running = true
    private var seqA = 0L
    private var seqG = 0L
    private var count = 0L
    private var queueFull = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        tv = TextView(this).apply { textSize = 18f; text = "starting..." }
        setContentView(tv)

        val ip = intent.getStringExtra("ip") ?: "192.168.0.10"
        val port = intent.getIntExtra("port", 5005)
        val periodUs = intent.getIntExtra("us", 10_000)

        thread(name = "udp-sender") {
            val socket = DatagramSocket()
            val addr = InetAddress.getByName(ip)
            val batch = ArrayList<String>(16)
            while (running) {
                val first = queue.poll(200, java.util.concurrent.TimeUnit.MILLISECONDS) ?: continue
                batch.clear(); batch.add(first)
                queue.drainTo(batch, 7)
                val bytes = batch.joinToString("\n").toByteArray()
                socket.send(DatagramPacket(bytes, bytes.size, addr, port))
            }
            socket.close()
        }

        sm = getSystemService(SENSOR_SERVICE) as SensorManager
        val acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        tv.text = "to $ip:$port, period ${periodUs}us\nacc=${acc?.name}\ngyro=${gyr?.name}"
        acc?.let { sm.registerListener(this, it, periodUs) }
        gyr?.let { sm.registerListener(this, it, periodUs) }
    }

    override fun onSensorChanged(e: SensorEvent) {
        val line = when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> "A,${seqA++},${e.timestamp},${e.values[0]},${e.values[1]},${e.values[2]}"
            Sensor.TYPE_GYROSCOPE -> "G,${seqG++},${e.timestamp},${e.values[0]},${e.values[1]},${e.values[2]}"
            else -> return
        }
        if (!queue.offer(line)) queueFull++
        if (++count % 200 == 0L) {
            tv.text = "events=$count  accSeq=$seqA  gyrSeq=$seqG  queueFull=$queueFull"
        }
    }

    override fun onAccuracyChanged(s: Sensor?, a: Int) {}

    override fun onDestroy() {
        running = false
        sm.unregisterListener(this)
        super.onDestroy()
    }
}
