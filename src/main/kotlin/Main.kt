import gurux.common.*
import gurux.io.BaudRate
import gurux.io.Parity
import gurux.io.StopBits
import gurux.serial.GXSerial
import java.nio.charset.StandardCharsets
import kotlin.math.*

private fun convertToUTM(latitude: Double, longitude: Double, north: Boolean, west: Boolean): Array<Any> {
    val k0 = 0.9996
    val alpha0 = -0.052359877
    val e = 0.0067394967
    val a = 6378137

    //var phi = Math.toRadians(latitude / 100.0)
    //var lamb = Math.toRadians(longitude / 100.0)
    var phi = Math.toRadians(convert_num_fixed(latitude))
    var lamb = Math.toRadians(convert_num_fixed(longitude))
    if (west)
        lamb = -lamb
    if (!north)
        phi = -phi

    //-------------------------------------
    val N = a / sqrt(1- e * sin(phi).pow(2))
    val T = tan(phi).pow(2)
    val C = e*cos(phi).pow(2)
    val A = cos(phi)*(lamb - (alpha0))
    val M = calculateM(a.toDouble(), e, phi)
    println("Latitude: $latitude - Longitude: $longitude")
    println("PHI: $phi - LAMBDA: $lamb")
    println("$N $T $C $A $M")
    val easting = k0 * N * (A + (((1 - T + C) * A.pow(3))/ 6) + (((5 - 18 * T + T.pow(2) + 72 * C - 58 * e) * A.pow(5)) / 120)) + 500000
    val northing = k0 * (M + N * tan(phi) * (A.pow(2) / 2 + (5 - T + 9 * C + 4 * C.pow(2)) * A.pow(4) / 24 + (61 - 58 * T + T.pow(2) + 600 * C - 330 * e) * A.pow(6) / 720))

    return arrayOf(easting, northing, 30)
}

fun calculateM(a: Double, eSquared: Double, phi: Double): Double {
    val term1 = 1 - eSquared / 4 - 3 * eSquared.pow(2) / 64 - 5 * eSquared.pow(3) / 256
    val term2 = 3 * eSquared / 8 + 3 * eSquared.pow(2) / 32 + 45 * eSquared.pow(3) / 1024
    val term3 = 15 * eSquared.pow(2) / 256 + 45 * eSquared.pow(3) / 1024
    val term4 = 35 * eSquared.pow(3) / 3072

    return a * (term1 * phi - term2 * sin(2 * phi) + term3 * sin(4 * phi) - term4 * sin(6 * phi))
}

fun convert_num_fixed(num: Double): Double {
    var num = num
    val grados = num.toInt() / 100
    val minutos = num - grados * 100
    num = grados + minutos / 60
    return num
}

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
            //println("Datos recibidos: $data")
            if (data.startsWith("\$GPGGA")) {
                val parts = data.split(",")
                // Parsear los datos de la sentencia GGA
                println(data)
                val time = parts[1]
                val latitude = parts[2]
                var north = false
                if (parts[3] == "N")
                    north = true
                var west = false
                if (parts[5] == "W")
                    west = true
                val longitude = parts[4]
                val altitude = parts[9]
                val arr = convertToUTM(latitude.toDouble(), longitude.toDouble(), north, west)
                println("UTM Coordinates: EASTING=${arr[0]}, NORTHING=${arr[1]}, Zone=${arr[2]}")
            }
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
        Thread.sleep(1000000) // Esperar 10 segundos para demostración
    } catch (e: Exception) {
        e.printStackTrace()
    } finally {
        gl.close()
        println("Puerto cerrado.")
    }
}