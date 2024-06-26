
import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import javafx.application.Application
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.control.Label
import javafx.scene.image.Image
import javafx.scene.image.ImageView
import javafx.scene.layout.StackPane
import javafx.scene.layout.VBox
import javafx.scene.paint.Color
import javafx.scene.text.Font
import javafx.stage.Stage
import java.io.File
import java.io.FileInputStream
import kotlin.math.*


val FILE_PATH = "coords.txt"

val imageWidthPixels = 1280 // Ancho de la imagen en píxeles
val imageHeightPixels = 720 // Alto de la imagen en píxeles
val groundCoverageWidthMeters = 442.0 // Cobertura en el suelo en metros (debes ajustar este valor)
val groundCoverageHeightMeters = 244.0

var lastLat: Double? = null
var lastLon: Double? = null
var lastTime: Long? = null  // Tiempo en milisegundos

var lastIndex = 0

data class UTMCoord(val northing: Double, val easting: Double, val speedLimit: Double)
class MapViewer : Application() {
    private lateinit var graphicsContext: GraphicsContext
    private lateinit var speedLabel: Label
    private lateinit var speedPane: StackPane

    // Singleton instance
    companion object {
        private var instance: MapViewer? = null

        fun getInstance(): MapViewer? {
            return instance
        }
    }

    override fun start(primaryStage: Stage) {
        instance = this

        val mainStack = StackPane()  // StackPane principal para permitir la superposición de elementos

        val root = VBox()  // Contenedor vertical para el mapa y el indicador de velocidad
        root.minWidth = 1280.0
        root.minHeight = 720.0

        // Configuración del canvas del mapa
        val canvas = Canvas(1280.0, 620.0)  // Ajustar el tamaño para dejar espacio al indicador de velocidad
        graphicsContext = canvas.graphicsContext2D
        val mapImage = Image(FileInputStream("/Users/ivangarcia/Documents/insia.jpg"))
        graphicsContext.drawImage(mapImage, 0.0, 0.0)

        // Configuración del indicador de velocidad
        val speedLabel = Label("0 km/h").apply {
            font = Font.font(24.0)
            textFill = Color.BLACK
        }
        val speedPane = StackPane(speedLabel).apply {
            style = "-fx-background-color: green;"  // Color inicial verde
            minHeight = 100.0  // Altura específica para el indicador
            minWidth = 1280.0  // Utiliza el ancho completo
        }

        root.children.addAll(canvas, speedPane)  // Añade ambos componentes al VBox

        // Cargar y configurar la imagen de la leyenda
        val legendImage = Image(FileInputStream("/Users/ivangarcia/Desktop/legend.png"))
        val legendView = ImageView(legendImage)
        legendView.setFitWidth(200.0)  // Ajustar el tamaño según sea necesario
        legendView.setFitHeight(100.0)
        legendView.setPreserveRatio(true)
        StackPane.setAlignment(legendView, javafx.geometry.Pos.TOP_RIGHT)  // Posicionar la leyenda en la esquina superior derecha

        mainStack.children.add(root)  // Añade el VBox al StackPane principal
        mainStack.children.add(legendView)  // Añade la leyenda sobre el VBox

        primaryStage.scene = Scene(mainStack, 1280.0, 720.0)
        primaryStage.title = "Map and Speed Viewer"
        primaryStage.show()
    }


    fun addMarker(pixelX: Double, pixelY: Double) {
        Platform.runLater {
            graphicsContext.fill = Color.RED
            graphicsContext.fillOval(pixelX - 5, pixelY - 5, 10.0, 10.0)
        }
    }

    fun addMarkerImproved(pixelX: Double, pixelY: Double, markerType: Int) {
        Platform.runLater {
            when (markerType) {
                1 -> { // Cuadrado rosa
                    graphicsContext.fill = Color.PINK
                    graphicsContext.fillRect(pixelX - 5, pixelY - 5, 10.0, 10.0)
                }
                2 -> { // Triángulo amarillo
                    graphicsContext.fill = Color.YELLOW
                    val xPoints = doubleArrayOf(pixelX, pixelX - 5, pixelX + 5)
                    val yPoints = doubleArrayOf(pixelY - 5, pixelY + 5, pixelY + 5)
                    graphicsContext.fillPolygon(xPoints, yPoints, 3)
                }
                3 -> { // Cruz azul
                    graphicsContext.stroke = Color.BLUE
                    graphicsContext.lineWidth = 2.0
                    graphicsContext.strokeLine(pixelX - 5, pixelY, pixelX + 5, pixelY)
                    graphicsContext.strokeLine(pixelX, pixelY - 5, pixelX, pixelY + 5)
                }
                4 -> { // Cruz morada
                    graphicsContext.stroke = Color.PURPLE
                    graphicsContext.lineWidth = 2.0
                    graphicsContext.strokeLine(pixelX - 5, pixelY, pixelX + 5, pixelY)
                    graphicsContext.strokeLine(pixelX, pixelY - 5, pixelX, pixelY + 5)
                }
                5 -> { // Punto rojo oscuro
                    graphicsContext.fill = Color.DARKRED
                    graphicsContext.fillOval(pixelX - 5, pixelY - 5, 10.0, 10.0)
                }
            }
        }
    }

    fun updateSpeedDisplay(speed: Double, status: String) {
        Platform.runLater {
            speedLabel.text = "$speed km/h"
            val color = when (status) {
                "green" -> "green"
                "yellow" -> "yellow"
                "red" -> "red"
                else -> "gray"
            }
            speedPane.style = "-fx-background-color: $color;"
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

fun calculateDistance(coord1: UTMCoord, coord2: UTMCoord): Double {
    return Math.sqrt(Math.pow(coord2.easting - coord1.easting, 2.0) + Math.pow(coord2.northing - coord1.northing, 2.0))
}

fun findClosestUTMCoord(target: UTMCoord, coords: List<UTMCoord>): UTMCoord {
    return coords.minByOrNull { calculateDistance(it, target) } ?: error("La lista de coordenadas no puede estar vacía")
}

fun calculateHaversineDistance(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
    val R = 6371.0  // Radio de la Tierra en kilómetros
    val dLat = Math.toRadians(lat2 - lat1)
    val dLon = Math.toRadians(lon2 - lon1)
    val a = sin(dLat / 2).pow(2) + cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2)
    val c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

}

fun updatePositionAndCalculateSpeed(lat: Double, lon: Double, timeElapsedSeconds: Double): Double {
    var speed = 0.0  // Inicializa speed en caso de que no se pueda calcular

    if (lastLat != null && lastLon != null && lastTime != null && timeElapsedSeconds > 0) {
        val distance = calculateHaversineDistance(lat, lon, lastLat!!, lastLon!!)  // Distancia en kilómetros
        speed = distance / (timeElapsedSeconds / 3600.0)  // Velocidad en km/h (distancia en km, tiempo en horas)
    }

    // Actualizar las últimas coordenadas
    lastLat = lat
    lastLon = lon
    lastTime = System.currentTimeMillis()

    return round(speed)
}



fun getSpeedStatus(currentSpeed: Double, speedLimit: Double): String {
    val lowerBound = speedLimit * 0.9  // 90% del límite de velocidad
    val upperBound = speedLimit * 1.1  // 110% del límite de velocidad

    println("Current Speed: $currentSpeed, Lower Bound: $lowerBound, Upper Bound: $upperBound")

    return when {
        currentSpeed < lowerBound -> "green"
        currentSpeed > upperBound -> "red"
        else -> "yellow"
    }
}

fun calculateClosestPoint(latitude: Double, longitude: Double, list: ArrayList<LatLonPoint>): LatLonPoint {
    var finalPoint = LatLonPoint(PointCoordinates(0.0, 0.0), 0.0, 0)
    var distance = 10000000.0
    var currDist: Double
    for (point in list) {
        currDist = sqrt((point.point.latitude - latitude).pow(2) + (point.point.longitude - longitude).pow(2))

        if (currDist < distance) {
            distance = currDist
            finalPoint = point
        }
        println("$currDist | $distance | ${finalPoint.point}")
    }
    return finalPoint
}

fun drawDetectedInsiaMap(list: ArrayList<LatLonPoint>){
    for (point in list) {
        val (lat, lon) = latLonToPixel(point.point.latitude, point.point.longitude,   40.386616,  -3.631593)
        if (point.speed == 10.0) {
            MapViewer.getInstance()?.addMarkerImproved(lat.toDouble(), lon.toDouble(), 1 )
        }

        if (point.speed == 20.0) {
            MapViewer.getInstance()?.addMarkerImproved(lat.toDouble(), lon.toDouble(), 2 )
        }

        if (point.speed == 15.0) {
            MapViewer.getInstance()?.addMarkerImproved(lat.toDouble(), lon.toDouble(), 3 )
        }

        if (point.speed == 18.0) {
            MapViewer.getInstance()?.addMarkerImproved(lat.toDouble(), lon.toDouble(), 4 )
        }

        if (point.speed == 5.0) {
            MapViewer.getInstance()?.addMarkerImproved(lat.toDouble(), lon.toDouble(), 5 )
        }

    }
}

fun comprobarSentidoAntiHorario(currentIndex: Int, lastIndex: Int, max: Int): Boolean {
    val window = 5
    // ANTIHORARIO
    if (lastIndex <= currentIndex/* && lastIndex <= max - window*/) {
        return true
    }
    if (lastIndex > currentIndex && lastIndex > max - window) {
        return true
    }
    // HORARIO
    if (lastIndex < currentIndex && lastIndex < max - window) {
        return false
    }
    if (lastIndex >= currentIndex/* && lastIndex >= window*/) {
        return false
    }
    return false
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



    val coordsList = File(FILE_PATH).readLines().map { line ->
        val (northing, easting, speedLimit) = line.split("\\s+".toRegex()).map { it.toDouble() }
        UTMCoord(northing, easting, speedLimit)
    }

    // Coordenadas de ejemplo para buscar el punto más cercano

    comPort.addDataListener(object : SerialPortDataListener {
        override fun getListeningEvents(): Int = SerialPort.LISTENING_EVENT_DATA_AVAILABLE

        override fun serialEvent(event: SerialPortEvent?) {
            if (event?.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE) return
            val newData = ByteArray(comPort.bytesAvailable())
            val numRead = comPort.readBytes(newData, newData.size.toLong())
            val dataString = String(newData, 0, numRead)
            println("Datos recibidos: $dataString")

            if (dataString.startsWith("\$GPGGA")) {
                val currentTime = System.currentTimeMillis()
                var timeDifference = 0.0;
                if (lastTime != null) {
                    timeDifference = (currentTime - lastTime!!) / 1000.0 // Time difference in seconds
                    println("Time since last data: $timeDifference seconds")
                }
                lastTime = currentTime

                val parts = dataString.split(",")
                // Parsear los datos de la sentencia GGA
                println(dataString)
                val latitude = parts[2].toDouble()
                val longitude = parts[4].toDouble()
                val decimalLat = convertToDecimal(latitude)
                val decimalLon = -convertToDecimal(longitude) // Negativo porque es Oeste

                // Calcular punto más cercano LAT LON
                val pointCoordinatesList: ArrayList<LatLonPoint> = ArrayList()
                //coordsList.forEach { pointCoordinatesList.add(LatLonPoint(UtmCoordinate(30, 'S', it.easting, it.northing).utmToPointCoordinates(), it.speedLimit)) }
                var i = 1
                for (coord in coordsList) {
                    pointCoordinatesList.add(LatLonPoint(UtmCoordinate(30, 'S', coord.easting, coord.northing).utmToPointCoordinates(), coord.speedLimit, i))
                    i++
                }
                val test_closesspoint = calculateClosestPoint(40.386113, -3.634501, pointCoordinatesList)
                println("Latitude: ${test_closesspoint.point.latitude} | Longitude: ${test_closesspoint.point.longitude} | SpeedLimit: ${test_closesspoint.speed}")
                val closestpoint = calculateClosestPoint(decimalLat, decimalLon, pointCoordinatesList)
                println("Latitude: ${closestpoint.point.latitude} | Longitude: ${closestpoint.point.longitude} | SpeedLimit: ${closestpoint.speed}")

                drawDetectedInsiaMap(pointCoordinatesList)
                val north = parts[3] == "N"
                val west = parts[5] == "W"
                val arr = convertToUTM(latitude, longitude, north, west)
                println("UTM Coordinates: EASTING=${arr[0]}, NORTHING=${arr[1]}, Zone=${arr[2]}")
                val easting_utm = arr[0] as Double
                val northing_utm = arr[1] as Double
                
                println(latLonToPixel(decimalLat, decimalLon,   40.386616,  -3.631593))
                val (lat2, lon2) = latLonToPixel(decimalLat, decimalLon,   40.386616,  -3.631593)
                println("Decimal Latitude: $decimalLat")
                println("Decimal Longitude: $decimalLon")
                val targetCoord = UTMCoord(northing_utm,easting_utm,0.0)
                val closestCoord = findClosestUTMCoord(targetCoord, coordsList)
                val speed = updatePositionAndCalculateSpeed(decimalLat, decimalLon, timeDifference)
                println("Velocidad actual: $speed km/h")

                if (comprobarSentidoAntiHorario(closestpoint.index, lastIndex, pointCoordinatesList.size))
                    println("Sentido antihorario")
                else
                    println("Sentido horario")

                //val speedStatus = getSpeedStatus(speed, closestCoord.speedLimit)
                val speedStatus = getSpeedStatus(speed, closestpoint.speed)

                println("La coordenada UTM más cercana es: Northing: ${closestCoord.northing}, Easting: ${closestCoord.easting}, Speed Limit: ${closestCoord.speedLimit}")
                println("Estado de la velocidad: $speedStatus")

                // Guardamos el indice para comprobar el sentido de la marcha
                lastIndex = closestpoint.index

                Platform.runLater {
                    MapViewer.getInstance()?.addMarker(lat2.toDouble(), lon2.toDouble())
                    MapViewer.getInstance()?.updateSpeedDisplay(speed, speedStatus)
                }
            }
        }
    })

    // Lanzar la interfaz gráfica de usuario de JavaFX
    Application.launch(MapViewer::class.java)

}

class LatLonPoint(point: PointCoordinates, speed: Double, index: Int) {
    var point = point
    var speed = speed
    var index = index
}
