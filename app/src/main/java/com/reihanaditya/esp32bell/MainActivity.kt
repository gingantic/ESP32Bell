package com.reihanaditya.esp32bell

import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.ContentValues.TAG
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.os.Handler
import android.util.Log
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Button
import android.widget.EditText
import android.widget.Toast
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

class MainActivity : AppCompatActivity() {
    // set variable null
    private lateinit var webView: WebView
    private lateinit var webSocketClient: MyWebSocketClient
    private lateinit var btnpintu: Button
    private lateinit var btnstart: Button

    private val webSocketListener = object : WebSocketListener() {
        override fun onMessage(webSocket: WebSocket, text: String) {
            Log.d(TAG, "Main Activity Received a text message: $text")
            // Handle the incoming message
            Handler(mainLooper).post {
                if (text == "bell") {
                    sendNotification()
                } else if (text == "pintu_tutup") {
                    btnpintu.text = "Buka Pintu"
                } else if (text == "pintu_buka") {
                    btnpintu.text = "Tutup Pintu"
                }else if (text == "pintu_terbuka") {
                    Handler(mainLooper).post {
                        Toast.makeText(this@MainActivity, "Pintu Masih Terbuka, Tolong Tutup", Toast.LENGTH_LONG).show()
                    }
                }
            }
        }

        override fun onOpen(webSocket: WebSocket, response: Response) {
            Log.d(TAG, "Main Activity WebSocket connection opened")
            Handler(mainLooper).post {
                btnpintu.isEnabled = true
                btnstart.isEnabled = true
                Toast.makeText(this@MainActivity, "Connected", Toast.LENGTH_SHORT).show()
            }
        }

        override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
            Log.d(TAG, "Main Activity WebSocket connection closed. Code: $code, Reason: $reason")
            Handler(mainLooper).post {
                btnpintu.isEnabled = false
                btnstart.text = "Start"
                Toast.makeText(this@MainActivity, "Disconnected", Toast.LENGTH_SHORT).show()
            }
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            Log.e(TAG, "Main Activity Error : ${t.message}")
            Handler(mainLooper).post {
                btnpintu.isEnabled = false
                btnstart.text = "Start"
                btnstart.isEnabled = true
                stopAll()
                Toast.makeText(this@MainActivity, "Fail", Toast.LENGTH_SHORT).show()
            }

        }
    }

    private val CHANNEL_ID = "ESP32Bell"
    private val notificationId = 1

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        val sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE)

        // initialize button dll
        btnstart = findViewById(R.id.btn_start)
        val edtip = findViewById<EditText>(R.id.edt_ipaddr)
        btnpintu = findViewById(R.id.btn_pintu)
        webView = findViewById(R.id.webView)

        // initialize notification channel
        createNotificationChannel()

        // initialize websocket
        webSocketClient = MyWebSocketClient(webSocketListener)

        // ngambil ip dari shared preferences
        val getValueSetting = sharedPreferences.getString("url_esp32", "")
        edtip.setText(getValueSetting)

        btnpintu.setOnClickListener {

            if (btnpintu.text == "Buka Pintu") {
                webSocketClient.sendMessage("buka")
            } else {
                webSocketClient.sendMessage("tutup")
            }
        }

        // btnstart untuk mulai
        btnstart.setOnClickListener {
            // check if edit text is empty
            btnstart.isEnabled = false

            if (edtip.text.toString().isEmpty()) {
                // set error message
                edtip.error = "IP Address is required"
                return@setOnClickListener
            }

            if (btnstart.text == "Start") {
                btnstart.text = "Stop"
            } else {
                btnstart.text = "Start"
                // stop websocket
                stopAll()
                return@setOnClickListener
            }

//            val wsUrl = "wss://free.blr2.piesocket.com/v3/1?api_key=Oore1KgwgzKJ7Xnjul9mMJ5YdwyP0cl9ANWdhqdD&notify_self=1"
//            webSocketClient.startWebSocket(wsUrl)
            var ip = edtip.text.toString()
            val streamUrl = "http://$ip/1"
            val wsUrl = "ws://$ip:81/"

            webSocketClient.startWebSocket(wsUrl)
            setWebView(streamUrl)

            val editor = sharedPreferences.edit()
            editor.putString("url_esp32", edtip.text.toString())
            editor.apply()
        }


    }

    private fun stopAll() {
        webSocketClient.cancelWebSocket()
        webView.stopLoading()
    }

    private fun setWebView(url: String) {
        // set webview
        webView.settings.javaScriptEnabled = true
        webView.settings.domStorageEnabled = true
        webView.settings.loadsImagesAutomatically = true
        webView.settings.cacheMode = android.webkit.WebSettings.LOAD_DEFAULT
        webView.settings.userAgentString = "Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:109.0) Gecko/20100101 Firefox/119.0"
        webView.settings.allowFileAccess = true
        webView.settings.allowContentAccess = true
        webView.settings.builtInZoomControls = true
        webView.settings.useWideViewPort = true

        webView.webViewClient = object : WebViewClient() {
            override fun shouldOverrideUrlLoading(view: WebView?, request: String?): Boolean {
                view?.loadUrl(url)
                return true
            }

            override fun onReceivedError(
                view: WebView?,
                errorCode: Int,
                description: String?,
                failingUrl: String?
            ) {
                // make toast if error
                Toast.makeText(this@MainActivity, "Error", Toast.LENGTH_SHORT).show()
            }
        }

        // Load the specified URL
        webView.loadUrl(url)
    }

    private fun createNotificationChannel() {
        // Create the NotificationChannel, but only on API 26+ because
        // the NotificationChannel class is new and not in the support library
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
            val name = "ESP32Bell"
            val descriptionText = "ESP32Bell Notification"
            val importance = NotificationManager.IMPORTANCE_DEFAULT
            val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
                description = descriptionText
            }

            // Register the channel with the system
            val notificationManager: NotificationManager =
                getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager

            notificationManager.createNotificationChannel(channel)
        }
    }

    private fun sendNotification() {
        // Check if the app has the necessary permission
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.POST_NOTIFICATIONS
            ) == PackageManager.PERMISSION_GRANTED
        ) {
            val intent = Intent(this, MainActivity::class.java).apply {
                flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
            }

            val pendingIntent: PendingIntent = PendingIntent.getActivity(this, 0, intent,
                PendingIntent.FLAG_IMMUTABLE)

            // Permission is granted, proceed with creating and sending the notification
            val builder = NotificationCompat.Builder(this, CHANNEL_ID)
                .setSmallIcon(R.drawable.ic_launcher_foreground)
                .setContentTitle("ESP32Bell")
                .setContentText("Bell di Tekan\nCek Pintu")
                .setContentIntent(pendingIntent)
                .setPriority(NotificationCompat.PRIORITY_DEFAULT)

            with(NotificationManagerCompat.from(this)) {
                // notificationId is a unique int for each notification that you must define
                notify(notificationId, builder.build())
            }
        } else {
            Log.e(TAG, "Permission not granted to send notifications")
        }
    }
}