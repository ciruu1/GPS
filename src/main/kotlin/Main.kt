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
import java.io.BufferedReader
import java.io.InputStreamReader


fun main() {
    val portName = "COM1" // Cambiar al puerto correcto en tu sistema
    val baudRate = 4800 // Cambiar a la velocidad correcta del puerto serie

    val gl = GXSerial()

    gl.baudRate = BaudRate.BAUD_RATE_115200
    gl.parity = Parity.NONE
    gl.portName = "COM4"
    gl.stopBits = StopBits.ONE
    gl.dataBits = 8

    val listener = GXSerialListener

    gl.addListener(listener);

    //val port = GXSerialPort(portName, baudRate)
    gl.open()

    gl.addListener(listener)


}

object GXSerialListener : IGXMediaListener {
    override fun onError(p0: Any?, p1: java.lang.Exception?) {
        TODO("Not yet implemented")
    }

    /**
     * Media component sends received data through this method.
     *
     * @param sender The source of the event.
     * @param e Event arguments.
     */
    override fun onReceived(sender: Any, e: ReceiveEventArgs) {
        //Handle received asyncronous data here.
    }

    override fun onMediaStateChange(p0: Any?, p1: MediaStateEventArgs?) {
        TODO("Not yet implemented")
    }

    override fun onTrace(p0: Any?, p1: TraceEventArgs?) {
        TODO("Not yet implemented")
    }

    override fun onPropertyChanged(p0: Any?, p1: PropertyChangedEventArgs?) {
        TODO("Not yet implemented")
    }

}