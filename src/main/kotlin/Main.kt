/*

import com.fazecast.jSerialComm.SerialPort
import java.io.BufferedReader
import java.io.InputStreamReader

fun main() {
    val portName = "COM4" // Cambiar al puerto correcto en tu sistema
    val baudRate = 115200 // Cambiar a la velocidad correcta del puerto serie

    val serialPort = SerialPort.getCommPort(portName)
    serialPort.baudRate = baudRate
    serialPort.numDataBits = 8
    serialPort.numStopBits = 1
    serialPort.parity = SerialPort.NO_PARITY
    serialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED)

    if (!serialPort.openPort()) {
        println("Error: No se pudo abrir el puerto serie")
        return
    }

    val input = BufferedReader(InputStreamReader(serialPort.inputStream))

    try {
        var line: String?
        while (input.readLine().also { line = it } != null) {
            if (line!!.startsWith("\$GPGGA")) {
                val parts = line!!.split(",")
                // Parsear los datos de la sentencia GGA
                val time = parts[1]
                val latitude = parts[2]
                val longitude = parts[4]
                val altitude = parts[9]
                println("Tiempo: $time, Latitud: $latitude, Longitud: $longitude, Altitud: $altitude")
            }
        }
    } catch (e: Exception) {
        e.printStackTrace()
    } finally {
        serialPort.closePort()
    }
}

*/
import gurux.common.*
import gurux.io.BaudRate
import gurux.io.Parity
import gurux.io.StopBits
import gurux.serial.GXSerial
import java.nio.charset.StandardCharsets

fun main() {
    val portName = "COM4" // Asegúrate de cambiar esto al puerto correcto en tu sistema
    val baudRate = BaudRate.BAUD_RATE_115200 // Ajusta esto a la velocidad correcta de tu puerto serie

    val gl = GXSerial().apply {
        this.portName = portName
        this.baudRate = baudRate
        this.parity = Parity.NONE
        this.stopBits = StopBits.ONE
        this.dataBits = 8
    }

    val listener = object : IGXMediaListener {
        override fun onError(sender: Any?, e: Exception?) {
            println("Error: ${e?.message}")
        }

        override fun onReceived(sender: Any, e: ReceiveEventArgs) {
            val data = String(e.data as ByteArray, StandardCharsets.UTF_8)
            println("Datos recibidos: $data")
        }

        override fun onMediaStateChange(sender: Any?, e: MediaStateEventArgs?) {
            println("Estado del medio cambiado: ${e?.state}")
        }

        override fun onTrace(sender: Any?, e: TraceEventArgs?) {
            println("Trace: ${e?.data}")
        }

        override fun onPropertyChanged(sender: Any?, e: PropertyChangedEventArgs?) {
            println("Propiedad cambiada: ${e?.propertyName}")
        }
    }

    gl.addListener(listener)

    try {
        gl.open()
        println("Puerto abierto. Escuchando datos...")
        // La ejecución se detiene aquí para una aplicación de consola. En una aplicación real, necesitas una forma de mantener la aplicación ejecutándose.
        Thread.sleep(10000) // Esperar 10 segundos para demostración
    } catch (e: Exception) {
        e.printStackTrace()
    } finally {
        gl.close()
        println("Puerto cerrado.")
    }
}