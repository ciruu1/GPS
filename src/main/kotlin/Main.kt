
import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import javafx.application.Application
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.control.Button
import javafx.scene.control.Label
import javafx.scene.image.Image
import javafx.scene.layout.BorderPane
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Circle
import javafx.scene.shape.Rectangle
import javafx.scene.text.Font
import javafx.stage.Stage
import okhttp3.MediaType.Companion.toMediaTypeOrNull
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONObject
import java.io.*
import java.net.URL
import kotlin.math.*


val FILE_PATH = "coords.txt"

val imageWidthPixels = 1280 // Ancho de la imagen en píxeles
val imageHeightPixels = 720 // Alto de la imagen en píxeles
var groundCoverageWidthMeters = 1161.792 // Cobertura en el suelo en metros (debes ajustar este valor)
var groundCoverageHeightMeters = 653.184

var lastLat: Double? = null
var lastLon: Double? = null
var lastTime: Long? = null  // Tiempo en milisegundos

var INSIA = false
var maptype = "satellite"
var zoom = 16
var lastIndex = 0

var historyList = ArrayList<PointCoordinates>()
data class OverpassElement(
    val type: String,
    val id: Long,
    val lat: Double? = null,
    val lon: Double? = null,
    val nodes: List<Long>? = null,
    val tags: Map<String, String>? = null
)

data class OverpassResponse(
    val elements: List<OverpassElement>
)

class OverpassApiClient {
    private val client = OkHttpClient()

    fun getNearestRoad(lat: Double, lon: Double): OverpassResponse {
        val query = """
            [out:json];
            (way(around:25,$lat,$lon)["highway"];
            );
            out body;
            >;
            out skel qt;
        """.trimIndent()

        val url = "http://overpass-api.de/api/interpreter?data=${query}"


        val request = Request.Builder()
            .url(url)
            .build()

        val response = client.newCall(request).execute()
        if (!response.isSuccessful) throw IOException("Unexpected code $response")

        val responseText = response.body?.string() ?: throw IOException("Response body is null")
        return parseResponse(responseText)
    }

    private fun parseResponse(responseText: String): OverpassResponse {
        val jsonObject = JSONObject(responseText)
        val elementsArray = jsonObject.getJSONArray("elements")
        val elements = mutableListOf<OverpassElement>()

        for (i in 0 until elementsArray.length()) {
            val elementObject = elementsArray.getJSONObject(i)
            val type = elementObject.getString("type")
            val id = elementObject.getLong("id")
            val lat = elementObject.optDouble("lat", Double.NaN).takeIf { !it.isNaN() }
            val lon = elementObject.optDouble("lon", Double.NaN).takeIf { !it.isNaN() }
            val nodes = elementObject.optJSONArray("nodes")?.let { jsonArray ->
                List(jsonArray.length()) { index -> jsonArray.getLong(index) }
            }
            val tags = elementObject.optJSONObject("tags")?.let { jsonObject ->
                jsonObject.keys().asSequence().associateWith { jsonObject.getString(it) }
            }

            elements.add(OverpassElement(type, id, lat, lon, nodes, tags))
        }

        return OverpassResponse(elements)
    }
}


data class TelegramMessage(
    val chat_id: String,
    val text: String? = null,
    val latitude: Double? = null,
    val longitude: Double? = null
)

fun sendTelegramMessage(botToken: String, chatId: String, messageText: String? = null, latitude: Double? = null, longitude: Double? = null) {
    val client = OkHttpClient()

    val message = JSONObject().apply {
        put("chat_id", chatId)
        messageText?.let { put("text", it) }
        latitude?.let { put("latitude", it) }
        longitude?.let { put("longitude", it) }
    }

    val messageBody = message.toString().toRequestBody("application/json; charset=utf-8".toMediaTypeOrNull())

    val url = when {
        messageText != null -> "https://api.telegram.org/bot$botToken/sendMessage"
        latitude != null && longitude != null -> "https://api.telegram.org/bot$botToken/sendLocation"
        else -> throw IllegalArgumentException("Message text or latitude and longitude must be provided")
    }

    val request = Request.Builder()
        .url(url)
        .post(messageBody)
        .build()

    try {
        client.newCall(request).execute().use { response ->
            if (!response.isSuccessful) throw IOException("Unexpected code $response")
            println("Message sent successfully.")
        }
    } catch (e: Exception) {
        println("Error sending message: ${e.message}")
    }
}



data class UTMCoord(val northing: Double, val easting: Double, val speedLimit: Double)
class MapViewer : Application() {
    private lateinit var graphicsContext: javafx.scene.canvas.GraphicsContext
    private lateinit var speedLabel: Label
    private lateinit var speedPane: StackPane
    private lateinit var infoLabel: Label
    private lateinit var infoPane: StackPane
    private lateinit var speedLimitLabel: Label
    private lateinit var speedLimitCircle: StackPane
    private lateinit var infoRectangle: StackPane
    private lateinit var mapCanvas: Canvas
    private lateinit var mapButton: Button
    private lateinit var plusButton: Button
    private lateinit var minusButton: Button
    private var mapCenterLat = 40.714728
    private var mapCenterLon = -73.998672

    companion object {
        private var instance: MapViewer? = null

        fun getInstance(): MapViewer? {
            return instance
        }

        fun getMapImageUrl(centerLat: Double, centerLon: Double): String {
            val apiKey = "AIzaSyDRiHr7s5yvIFYBySkctaXfZLSJHGZbTqQ"
            return "https://maps.googleapis.com/maps/api/staticmap?center=$centerLat,$centerLon&zoom=$zoom&size=640x360&scale=2&maptype=$maptype&key=$apiKey"
        }
    }

    fun changeMapType() {
        maptype = when (maptype) {
            "roadmap" -> "satellite"
            "satellite" -> "terrain"
            "terrain" -> "hybrid"
            else -> "roadmap"
        }
        updateMapCenter(mapCenterLat, mapCenterLon)
        mapButton.text = "$maptype"
    }

    private fun zoomIn() {
        zoom += 1
        updateGroundCoverage()
        // Implementar lógica para actualizar el zoom del mapa
        println("Zoom increased to $zoom")
        println("Ground coverage: $groundCoverageWidthMeters x $groundCoverageHeightMeters meters")
    }

    private fun zoomOut() {
        zoom -= 1
        updateGroundCoverage()
        // Implementar lógica para actualizar el zoom del mapa
        println("Zoom decreased to $zoom")
        println("Ground coverage: $groundCoverageWidthMeters x $groundCoverageHeightMeters meters")
    }

    private fun updateGroundCoverage() {
        val zoomFactor = Math.pow(2.0, (16 - zoom).toDouble())
        groundCoverageWidthMeters = 1123.0 * zoomFactor
        groundCoverageHeightMeters = 654.0 * zoomFactor
    }


    private fun getNextMapType(): String {
        return when (maptype) {
            "roadmap" -> "satellite"
            "satellite" -> "terrain"
            "terrain" -> "hybrid"
            else -> "roadmap"
        }
    }

    override fun start(primaryStage: Stage) {
        instance = this
        val root = StackPane()

        mapCanvas = Canvas(1280.0, 620.0)  // Ajustar el tamaño para dejar espacio al indicador de velocidad
        graphicsContext = mapCanvas.graphicsContext2D
        val mapImage = Image(FileInputStream("/Users/ivangarcia/Documents/insia.jpg"))
        graphicsContext.drawImage(mapImage, 0.0, 0.0)

        loadMapImage()

        speedLabel = Label("NA km/h").apply {
            font = Font.font(24.0)
            textFill = Color.BLACK
        }

        speedPane = StackPane(speedLabel).apply {
            style = "-fx-background-color: green;"
            minHeight = 100.0
            minWidth = 1280.0
        }

        infoLabel = Label().apply {
            font = Font.font(16.0)
            textFill = Color.WHITE
        }
        val rectangle = Rectangle(40.0, 20.0, Color.BLACK).apply {

        }

        infoPane = StackPane(rectangle, infoLabel).apply {
            style = "-fx-background-color: rgba(0, 0, 0, 0.7);"

        }

        infoLabel = Label("-").apply {
            font = Font.font(24.0)
            textFill = Color.WHITE

        }
        val rectange_info = Rectangle(400.0, 200.0).apply {
            opacity = 0.7
            arcWidth = 30.0
            arcHeight = 30.0

        }

        infoRectangle = StackPane(rectange_info, infoLabel).apply {
            translateX = -425.0
            translateY = -250.0
        }

        speedLimitLabel = Label("-").apply {
            font = Font.font(24.0)
            textFill = Color.BLACK
        }


        val circle = Circle(30.0, Color.WHITE).apply {
            stroke = Color.RED
            strokeWidth = 6.0
        }
        speedLimitCircle = StackPane(circle, speedLimitLabel).apply {
            translateX = -575.0
            translateY = 200.0
        }

        mapButton = Button(getNextMapType()).apply {
            setOnAction {

                changeMapType()
            }
            style = "-fx-background-color: rgba(255, 255, 255, 0.7);" // Background color with some transparency
        }

        plusButton = Button("+").apply {
            setOnAction {
                zoomIn()
                updateMapCenter(mapCenterLat, mapCenterLon)
            }
            style = "-fx-background-color: rgba(255, 255, 255, 0.7);" // Background color with some transparency
        }

        minusButton = Button("-").apply {
            setOnAction {
                zoomOut()
                updateMapCenter(mapCenterLat, mapCenterLon)
            }
            style = "-fx-background-color: rgba(255, 255, 255, 0.7);" // Background color with some transparency
        }

        val buttonBox = StackPane(mapButton).apply {
            translateX = -20.0  // Adjust X coordinate as needed
            translateY = 20.0   // Adjust Y coordinate as needed
        }

        val buttonBoxplus = StackPane(plusButton).apply {
            translateX = -90.0  // Adjust X coordinate as needed
            translateY = 20.0   // Adjust Y coordinate as needed
        }

        val buttonBoxminus = StackPane(minusButton).apply {
            translateX = -120.0  // Adjust X coordinate as needed
            translateY = 20.0   // Adjust Y coordinate as needed
        }

        val borderPane = BorderPane()
        borderPane.center = mapCanvas
        borderPane.bottom = speedPane

        root.children.addAll(borderPane, speedLimitCircle, infoRectangle, buttonBox, buttonBoxplus, buttonBoxminus)
        StackPane.setAlignment(speedLimitCircle, javafx.geometry.Pos.TOP_LEFT)
        StackPane.setAlignment(infoRectangle, javafx.geometry.Pos.TOP_LEFT)
        StackPane.setAlignment(mapButton, javafx.geometry.Pos.TOP_RIGHT)
        StackPane.setAlignment(plusButton, javafx.geometry.Pos.TOP_RIGHT)
        StackPane.setAlignment(minusButton, javafx.geometry.Pos.TOP_RIGHT)

        primaryStage.scene = Scene(root, 1280.0, 720.0)
        primaryStage.title = "Map and Speed Viewer"
        primaryStage.show()
    }


    fun loadMapImage() {
        Platform.runLater {
            val url = getMapImageUrl(mapCenterLat, mapCenterLon)
            val img = Image(URL(url).toString(), true)  // true para carga en segundo plano
            img.exceptionProperty().addListener { _, _, e ->
                println("Failed to load image: $e")
            }
            img.errorProperty().addListener { _, _, isError ->
                if (isError) {
                    println("Error loading image.")
                }
            }

            img.progressProperty().addListener { _, _, progress ->
                println("Image load progress: $progress")
            }

            img.heightProperty().addListener { _, _, _ ->
                if (img.isError.not()) {
                    graphicsContext.drawImage(img, 0.0, 0.0)
                }
            }

            for (point in historyList) {
                val (pixelX, pixelY) = latLonToPixel(point.latitude, point.longitude, mapCenterLat, mapCenterLon)
                MapViewer.getInstance()?.addMarker(pixelX.toDouble(), pixelY.toDouble())
            }
        }
    }

    fun downloadMapImage(centerLat: Double, centerLon: Double, filePath: String) {
        try {
            val url = getMapImageUrl(centerLat, centerLon)
            val imageUrl = URL(url)
            val connection = imageUrl.openConnection()
            val inputStream: InputStream = connection.getInputStream()
            val outputStream = FileOutputStream(filePath)

            val buffer = ByteArray(2048)
            var bytesRead: Int
            while (inputStream.read(buffer).also { bytesRead = it } != -1) {
                outputStream.write(buffer, 0, bytesRead)
            }
            outputStream.close()
            inputStream.close()
            println("Image downloaded successfully: $filePath")
        } catch (e: Exception) {
            e.printStackTrace()
            println("Error downloading map image: ${e.message}")
        }
    }


    fun updateMapCenter(newLat: Double, newLon: Double) {
        mapCenterLat = newLat
        mapCenterLon = newLon
        loadMapImage()
        downloadMapImage(mapCenterLat, mapCenterLon, "/Users/ivangarcia/Documents/insia.jpg")
        val mapImage = Image(FileInputStream("/Users/ivangarcia/Documents/insia.jpg"))
        graphicsContext.drawImage(mapImage, 0.0, 0.0)
    }
    fun getMapCenter(): Pair<Double, Double> {
        return Pair(mapCenterLat, mapCenterLon)
    }

    fun addMarker(pixelX: Double, pixelY: Double) {
        Platform.runLater {
            graphicsContext.fill = Color.RED
            graphicsContext.fillOval(pixelX - 2.4, pixelY - 2.5, 5.0, 5.0)
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

    private fun updateRoadInfoDisplay(info: Map<String, String>) {
        Platform.runLater {
            val infoText = info.entries
                .filter { it.value != "N/A" }
                .joinToString("\n") { "${it.key}: ${it.value}" }

            infoLabel.text = infoText
            infoPane.style = "-fx-background-color: rgba(255, 255, 255, 0.8);"
        }
    }

    private fun translateHighway(highway: String): String {
        return when (highway) {
            "motorway" -> "Autopista"
            "trunk" -> "Carretera Principal"
            "primary" -> "Carretera Primaria"
            "secondary" -> "Carretera Secundaria"
            "tertiary" -> "Poblado"
            "unclassified" -> "No Clasificada"
            "residential" -> "Residencial"
            "service" -> "Servicio"
            "living_street" -> "Calle Residencial"
            "pedestrian" -> "Peatonal"
            "track" -> "Camino"
            "bus_guideway" -> "Guía de Autobús"
            "escape" -> "Vía de Escape"
            "raceway" -> "Pista de Carreras"
            "road" -> "Carretera"
            else -> highway
        }
    }

    private fun translateLanes(lanes: String): String {
        return when (lanes) {
            "1" -> "Un carril"
            "2" -> "Dos carriles"
            "3" -> "Tres carriles"
            "4" -> "Cuatro carriles"
            else -> "$lanes carriles"
        }
    }

    private fun translateWay(lanes: String): String {
        return when (lanes) {
            "Yes" -> "Unico sentido"
            "No" -> "Doble sentido"
            else -> "Unico sentido"
        }
    }

    private fun getDefaultMaxSpeed(highway: String): String {
        return when (highway) {
            "motorway" -> "120" // Autopista
            "trunk" -> "100" // Carretera Principal
            "primary" -> "90" // Carretera Primaria
            "secondary" -> "80" // Carretera Secundaria
            "tertiary" -> "50" // Poblado
            "residential" -> "30" // Residencial
            "living_street" -> "20" // Calle Residencial
            else -> "50" // Default
        }
    }

    fun updateInfoPane(
        highway: String, intRef: String, lanes: String, maxspeed: String, name: String,
        oneway: String, ref: String, refColour: String, sourceName: String, surface: String,
        turnLanes: String, wikidata: String, wikipedia: String
    ) {
        val translatedHighway = translateHighway(highway)
        val translatedLanes = if (lanes != "N/A") translateLanes(lanes) else "N/A"
        val speed = if (maxspeed != "N/A") maxspeed else getDefaultMaxSpeed(highway)
        val translatedoneway = translateWay(oneway)

        val info = buildString {
            if (translatedHighway != "N/A") append("Tipo de via: $translatedHighway\n")
            if (translatedLanes != "N/A") append("Carriles: $translatedLanes\n")
            if (speed != "N/A") append("Limite de velocidad: $speed km/h\n")
            if (name != "N/A") append("Nombre: $name\n")
            if (oneway != "N/A") append("$translatedoneway\n")
            if (surface != "N/A") append("Superficie: $surface\n")
        }
        infoLabel.text = info
        speedLabel.text = "$speed km/h"
        speedLimitLabel.text = speed
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

var lastSpeed = 0.0

fun checkForAccident(
    botToken: String,
    chatId: String,
    currentSpeed: Double,
    latitude: Double,
    longitude: Double,
    highwayType: String
) {
    if (highwayType == "motorway" && (lastSpeed - currentSpeed) > 50) {
        val messageText = """
        ¡⚠️ Posible Accidente Detectado! 🚨
    
        Por favor, este es un aviso de emergencia. Notifique a las autoridades de emergencia inmediatamente.
    
        Recordatorio: Este es solo un aviso y carece de confirmación final. Se recomienda realizar una verificación adicional.
    
        🙏 ¡Gracias por su pronta atención y cooperación!
        
        📍 Ubicación del Vehículo:
        """.trimIndent()
        sendTelegramMessage(botToken, chatId, messageText)
        sendTelegramMessage(botToken, chatId, latitude = latitude, longitude = longitude)
    }
    lastSpeed = currentSpeed
}




fun main() {
    val portName = "/dev/tty.usbmodem1101"
    val baudRate = 115200

    val comPort = SerialPort.getCommPort(portName)
    comPort.baudRate = baudRate
    comPort.parity = SerialPort.NO_PARITY
    comPort.numStopBits = SerialPort.ONE_STOP_BIT
    comPort.numDataBits = 8
    val apiClient = OverpassApiClient()

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

    comPort.addDataListener(object : SerialPortDataListener {
        override fun getListeningEvents(): Int = SerialPort.LISTENING_EVENT_DATA_AVAILABLE

        override fun serialEvent(event: SerialPortEvent?) {
            if (event?.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE) return
            val newData = ByteArray(comPort.bytesAvailable())
            val numRead = comPort.readBytes(newData, newData.size.toLong())
            val dataString = String(newData, 0, numRead)
            if (dataString.startsWith("\$GPGGA")) {
                val parts = dataString.split(",")
                val latitude = convertToDecimal(parts[2].toDouble())
                val longitude = -convertToDecimal(parts[4].toDouble())  // Assuming Western Hemisphere
                val currentTime = System.currentTimeMillis()
                val timeDifference = (currentTime - (lastTime ?: currentTime)) / 1000.0
                lastTime = currentTime

                println("Current Position: $latitude, $longitude")

                if (historyList.size >= 50) {
                    historyList.removeFirst()
                }
                historyList.add(PointCoordinates(longitude, latitude))

                val mapCenter = MapViewer.getInstance()?.getMapCenter()
                val mapCenterLat = mapCenter?.first ?: return
                val mapCenterLon = mapCenter?.second ?: return

                val (pixelX, pixelY) = latLonToPixel(latitude, longitude, mapCenterLat, mapCenterLon)

                val speed = updatePositionAndCalculateSpeed(latitude, longitude, timeDifference)
                println("Velocidad actual: $speed km/h")

                // Calcular punto más cercano LAT LON
                val pointCoordinatesList: ArrayList<LatLonPoint> = ArrayList()
                var i = 1
                for (coord in coordsList) {
                    pointCoordinatesList.add(LatLonPoint(UtmCoordinate(30, 'S', coord.easting, coord.northing).utmToPointCoordinates(), coord.speedLimit, i))
                    i++
                }
                val closestpoint = calculateClosestPoint(latitude, longitude, pointCoordinatesList)
                if (calculateHaversineDistance(closestpoint.point.latitude, closestpoint.point.longitude, latitude, longitude) < 10.0)
                    INSIA = true


                val north = parts[3] == "N"
                val west = parts[5] == "W"
                val arr = convertToUTM(parts[2].toDouble(), parts[4].toDouble(), north, west)
                println("UTM Coordinates: EASTING=${arr[0]}, NORTHING=${arr[1]}, Zone=${arr[2]}")

                val speedStatus = getSpeedStatus(speed, closestpoint.speed)
                Platform.runLater {
                    MapViewer.getInstance()?.addMarker(pixelX.toDouble(), pixelY.toDouble())
                    if (INSIA) {
                        MapViewer.getInstance()?.updateSpeedDisplay(speed, speedStatus)
                    }else{
                        MapViewer.getInstance()?.updateSpeedDisplay(speed, "gray")
                    }
                    val (pixelX2, pixelY2) = latLonToPixel(40.54355924142748, -3.6252657563084014, latitude, longitude)
                    MapViewer.getInstance()?.addMarker(pixelX2.toDouble(), pixelY2.toDouble())
                    try {
                        val response = apiClient.getNearestRoad(latitude, longitude)
                        response.elements.forEach { element ->
                            if(element.type == "way"){
                                println("Type: ${element.type}, ID: ${element.id}, Lat: ${element.lat}, Lon: ${element.lon}, Tags: ${element.tags}")
                                // Asignar tags a variables
                                val tags = element.tags as Map<String, String>
                                val highway = tags["highway"] ?: "N/A"
                                val intRef = tags["int_ref"] ?: "N/A"
                                val lanes = tags["lanes"] ?: "N/A"
                                val maxspeed = tags["maxspeed"] ?: "N/A"
                                val name = tags["name"] ?: "N/A"
                                val oneway = tags["oneway"] ?: "N/A"
                                val ref = tags["ref"] ?: "N/A"
                                val refColour = tags["ref:colour"] ?: "N/A"
                                val sourceName = tags["source:name"] ?: "N/A"
                                val surface = tags["surface"] ?: "N/A"
                                val turnLanes = tags["turn:lanes"] ?: "N/A"
                                val wikidata = tags["wikidata"] ?: "N/A"
                                val wikipedia = tags["wikipedia"] ?: "N/A"

                                // Mostrar los detalles por pantalla
                                println("Highway: $highway")
                                println("Int_ref: $intRef")
                                println("Lanes: $lanes")
                                println("Maxspeed: $maxspeed")
                                println("Name: $name")
                                println("Oneway: $oneway")
                                println("Ref: $ref")
                                println("Ref (colour): $refColour")
                                println("Source (name): $sourceName")
                                println("Surface: $surface")
                                println("Turn (lanes): $turnLanes")
                                println("Wikidata: $wikidata")
                                println("Wikipedia: $wikipedia")



                                MapViewer.getInstance()?.updateInfoPane(highway, intRef, lanes, maxspeed, name, oneway, ref, refColour, sourceName, surface, turnLanes, wikidata, wikipedia)
                                checkForAccident("6982141211:AAGIRNGP_GlnRwb3afpOmTuQU6chTqTiaIE","-4218279163",speed,latitude, longitude, highway)
                            }
                        }

                    } catch (e: Exception) {
                        e.printStackTrace()
                    }

                }

                // Check if the user is near the edge of the map and update map center proactively
                if (shouldUpdateMap(pixelX, pixelY)) {
                    Platform.runLater {
                        MapViewer.getInstance()?.updateMapCenter(latitude, longitude)
                        // Re-draw all markers on the new map center
                        for (point in historyList) {
                            val (pixelX, pixelY) = latLonToPixel(point.latitude, point.longitude, latitude, longitude)
                            MapViewer.getInstance()?.addMarker(pixelX.toDouble(), pixelY.toDouble())
                        }
                    }
                }
            }
        }
    })

    Application.launch(MapViewer::class.java)
}

fun shouldUpdateMap(pixelX: Int, pixelY: Int): Boolean {
    val updateThreshold = 100  // threshold in pixels from the edge of the map
    return pixelX < updateThreshold || pixelX > imageWidthPixels - updateThreshold ||
            pixelY < updateThreshold || pixelY > imageHeightPixels - updateThreshold
}


class LatLonPoint(point: PointCoordinates, speed: Double, index: Int) {
    var point = point
    var speed = speed
    var index = index
}