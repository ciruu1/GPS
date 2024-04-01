
import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import javafx.application.Application
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.StackPane
import javafx.stage.Stage
import java.io.FileInputStream
import kotlin.math.*


val imageWidthPixels = 1280 // Ancho de la imagen en píxeles
val imageHeightPixels = 720 // Alto de la imagen en píxeles
val groundCoverageWidthMeters = 1140.0 // Cobertura en el suelo en metros (debes ajustar este valor)
val groundCoverageHeightMeters = 637.0

class MapViewer : Application() {

    private lateinit var graphicsContext: GraphicsContext
    companion object {
        private var instance: MapViewer? = null

        fun getInstance(): MapViewer? {
            return instance
        }
    }
    override fun start(primaryStage: Stage) {
        instance = this
        val root = StackPane()
        val canvas = Canvas(1280.0, 720.0) // Tamaño del canvas para la imagen del mapa
        root.children.add(canvas)
        graphicsContext = canvas.graphicsContext2D

        // Carga y muestra la imagen de mapa
        val mapImage = Image(FileInputStream("/Users/ivangarcia/Documents/casa4.jpg"))
        graphicsContext.drawImage(mapImage, 0.0, 0.0)

        primaryStage.scene = Scene(root)
        primaryStage.title = "Map Viewer"
        primaryStage.show()

        // Añade un marcador en el centro de la imagen
        val centerPixelX = canvas.width / 2
        val centerPixelY = canvas.height / 2
    }

    // Función para convertir latitud/longitud a píxeles y dibujar un marcador
    fun addMarker(pixelX: Double, pixelY: Double) {
        Platform.runLater {
            graphicsContext.apply {
                fill = javafx.scene.paint.Color.RED // Establece el color del relleno a rojo
                fillOval(pixelX - 5, pixelY - 5, 10.0, 10.0) // Dibuja un marcador
            }
        }
    }
}

// Función de conversión de latitud/longitud a píxeles (este código debe ser ajustado con tu lógica y escala)
fun latLonToPixel(lat: Double, lon: Double, centerLat: Double, centerLon: Double): Pair<Int, Int> {
    val metersPerLatDegree = 111320.0 // Más preciso para la latitud promedio del mundo
    val metersPerLonDegree = metersPerLatDegree * cos(Math.toRadians(centerLat)) // Ajustado por coseno de la latitud central

    val deltaLatMeters = (lat - centerLat) * metersPerLatDegree
    val deltaLonMeters = (lon - centerLon) * metersPerLonDegree

    val pixelX = (deltaLonMeters * imageWidthPixels / groundCoverageWidthMeters) + imageWidthPixels / 2
    val pixelY = imageHeightPixels / 2 - (deltaLatMeters * imageHeightPixels / groundCoverageHeightMeters) // Invertido Y para corregir el origen de coordenadas

    return Pair(pixelX.toInt(), pixelY.toInt())
}


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

fun convertToDecimal(degreesMinutes: Double): Double {
    val degrees = (degreesMinutes / 100).toInt()
    val minutes = degreesMinutes % 100
    return degrees + (minutes / 60)
}

fun convert_num_fixed(num: Double): Double {
    var num = num
    val grados = num.toInt() / 100
    val minutos = num - grados * 100
    num = grados + minutos / 60
    return num
}

fun main() {
    val portName = "/dev/tty.usbmodem1101" // Asegúrate de cambiar esto al puerto correcto en tu sistema
    val baudRate = 115200 // Ajusta esto a la velocidad correcta de tu puerto serie

    val comPort = SerialPort.getCommPort(portName)
    comPort.baudRate = baudRate
    comPort.parity = SerialPort.NO_PARITY
    comPort.numStopBits = SerialPort.ONE_STOP_BIT
    comPort.numDataBits = 8

    if (comPort.openPort()) {
        println("Puerto abierto exitosamente.")
    } else {
        println("No se pudo abrir el puerto.")
        return
    }

    comPort.addDataListener(object : SerialPortDataListener {
        override fun getListeningEvents(): Int = SerialPort.LISTENING_EVENT_DATA_AVAILABLE

        override fun serialEvent(event: SerialPortEvent?) {
            if (event?.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE) return
            val newData = ByteArray(comPort.bytesAvailable())
            val numRead = comPort.readBytes(newData, newData.size.toLong())
            val dataString = String(newData, 0, numRead)
            println("Datos recibidos: $dataString")
            // Implementa aquí tu lógica específica para manejar los datos recibidos
            if (dataString.startsWith("\$GPGGA")) {
                val parts = dataString.split(",")
                // Parsear los datos de la sentencia GGA
                println(dataString)
                val latitude = parts[2].toDouble()
                val longitude = parts[4].toDouble()
                val north = parts[3] == "N"
                val west = parts[5] == "W"
                val arr = convertToUTM(latitude, longitude, north, west)
                println("UTM Coordinates: EASTING=${arr[0]}, NORTHING=${arr[1]}, Zone=${arr[2]}")
                val decimalLat = convertToDecimal(latitude)
                val decimalLon = -convertToDecimal(longitude) // Negativo porque es Oeste
                println(latLonToPixel(decimalLat, decimalLon,  40.549455555555554, -3.62285))
                val (lat2, lon2) = latLonToPixel(decimalLat, decimalLon,  40.549455555555554, -3.62285)
                println("Decimal Latitude: $decimalLat")
                println("Decimal Longitude: $decimalLon")
                Platform.runLater {
                    MapViewer.getInstance()?.addMarker(lat2.toDouble(), lon2.toDouble())
                }
            }
        }
    })

    // Lanzar la interfaz gráfica de usuario de JavaFX
    Application.launch(MapViewer::class.java)

}