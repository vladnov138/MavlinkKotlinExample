package org.example

import io.dronefleet.mavlink.MavlinkConnection
import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.ardupilotmega.ArdupilotmegaDialect
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.GlobalPositionInt
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.minimal.Heartbeat
import io.dronefleet.mavlink.minimal.MavAutopilot
import io.dronefleet.mavlink.minimal.MavState
import io.dronefleet.mavlink.minimal.MavType
import kotlinx.coroutines.*
import java.io.IOException
import java.io.OutputStream
import java.io.PipedInputStream
import java.io.PipedOutputStream
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketAddress
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

@Volatile
var currentRemoteAddress: SocketAddress? = null

suspend fun main(): Unit = coroutineScope {
    val defaultRemoteAddress: SocketAddress = InetSocketAddress("192.168.0.15", 14550)
    val localPort = 14550
    val bufferSize = 65535

    // Socket for receiving udp data from the SITL
    val udpSocket = DatagramSocket(localPort)

    val pipedInputStream = PipedInputStream()
    val pipedOutputStream = PipedOutputStream(pipedInputStream)

    // Custom OutputStream for sending data to the SITL
    val udpOut: OutputStream = object : OutputStream() {
        private val buffer = ByteArray(bufferSize)
        private var position = 0

        override fun write(b: Int) {
            write(byteArrayOf(b.toByte()), 0, 1)
        }

        @Synchronized
        override fun write(b: ByteArray, off: Int, len: Int) {
            if (position + len > buffer.size) flush()
            System.arraycopy(b, off, buffer, position, len)
            position += len
        }

        @Synchronized
        override fun flush() {
            if (position > 0) {
                // Detect dynamic SITL address
                val targetAddress = currentRemoteAddress ?: defaultRemoteAddress
                val packet = DatagramPacket(buffer, 0, position, targetAddress)
                println("Sending packet to ${packet.address.hostAddress}:${packet.port}")
                udpSocket.send(packet)
                position = 0
            }
        }
    }

    // Thread for receiving udp data
    val service: ExecutorService = Executors.newSingleThreadExecutor()
    service.execute {
        try {
            val packet = DatagramPacket(ByteArray(bufferSize), bufferSize)
            while (!udpSocket.isClosed) {
                udpSocket.receive(packet)
                // update remote address
                currentRemoteAddress = InetSocketAddress(packet.address, packet.port)
//                println("Received packet from ${packet.address.hostAddress}:${packet.port}")
//                println(packet.data.contentToString())
                pipedOutputStream.write(packet.data, packet.offset, packet.length)
                pipedOutputStream.flush()
            }
        } catch (e: IOException) {
            e.printStackTrace()
        } finally {
            try {
                pipedOutputStream.close()
            } catch (e: IOException) {
                e.printStackTrace()
            }
            if (!udpSocket.isClosed) {
                udpSocket.close()
            }
            service.shutdown()
        }
    }

    val mavlinkConnection = MavlinkConnection.builder(pipedInputStream, udpOut)
        .dialect(MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA, ArdupilotmegaDialect())
        .build()

    // Receiving messages
    launch {
        while (isActive) {
            val msg: MavlinkMessage<*>? = try {
                mavlinkConnection.next()
            } catch (e: Exception) {
                e.printStackTrace()
                null
            }
            if (msg != null) {
//                println("Received message: systemId=${msg.originSystemId}, componentId=${msg.originComponentId}, payload=${msg.payload}")
                if (msg.payload is GlobalPositionInt) {
                    val globalPosition = msg as MavlinkMessage<GlobalPositionInt>
                    println("Received global position: ${globalPosition}")
                    println("${globalPosition.payload.alt()}")
                    println("${globalPosition.payload.lat() / 1E7}")
                    println("${globalPosition.payload.lon() / 1E7}")
                }
            }
        }
    }

    // send heartbeat GCS and ARM Command Long
    launch {
        while (isActive) {
            val heartbeat = Heartbeat.builder()
                .type(MavType.MAV_TYPE_GCS)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_INVALID)
                .systemStatus(MavState.MAV_STATE_ACTIVE)
                .mavlinkVersion(3)
                .build()
            println("Sending heartbeat from GCS")
            mavlinkConnection.send2(255, 0, heartbeat)
            udpOut.flush()
            delay(1000)

            val armCommand = CommandLong.builder()
                .targetSystem(1)
                .targetComponent(1)
                .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
                .confirmation(0)
                .param1(1F)  // 1 = ARM
                .param2(0F)
                .param3(0F)
                .param4(0F)
                .param5(0F)
                .param6(0F)
                .param7(0F)
                .build()
            println("Sending ARM command")
//            mavlinkConnection.send2(255, 0, armCommand)
            udpOut.flush()
            delay(5000)
        }
    }

    awaitCancellation()
}
