package com.reihanaditya.esp32bell

import okhttp3.*

class MyWebSocketClient(private val listener: WebSocketListener) {

    private val client = OkHttpClient()
    lateinit var websocketclient: WebSocket

    fun startWebSocket(url: String) {
        val request = Request.Builder()
            .url(url)
            .build()

        websocketclient = client.newWebSocket(request, listener)
    }

    fun sendMessage(message: String) {
        websocketclient.send(message)
    }

    fun checkWebSocket() : Boolean {
        if (websocketclient == null)
            return false
        return websocketclient.send("ping")
    }

    fun cancelWebSocket() {
        client.dispatcher.cancelAll()
    }
}
