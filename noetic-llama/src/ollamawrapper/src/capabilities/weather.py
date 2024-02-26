import os
import dotenv
import geocoder
import requests 

if os.path.exists(os.path.join(os.path.dirname(__file__), "..", "..", "..", "apikeys.env")):
    dotenv.load_dotenv(os.path.join(os.path.dirname(__file__), "..", "..", "..", "apikeys.env"))

if "BINGMAPS" not in os.environ:
    raise Exception("'BINGMAPS' API key environment variable not found")

WEATHER_CODES = {
    0: "Clear sky",
    1: "Mainly clear, partly cloudy, and overcast",
    2: "Mainly clear, partly cloudy, and overcast",
    3: "Mainly clear, partly cloudy, and overcast",
    45: "Fog and depositing rime fog",
    48: "Fog and depositing rime fog",
    51: "Drizzle: Light, moderate, and dense intensity",
    53: "Drizzle: Light, moderate, and dense intensity",
    55: "Drizzle: Light, moderate, and dense intensity",
    56: "Freezing Drizzle: Light and dense intensity",
    57: "Freezing Drizzle: Light and dense intensity",
    61: "Rain: Slight, moderate and heavy intensity",
    63: "Rain: Slight, moderate and heavy intensity",
    65: "Rain: Slight, moderate and heavy intensity",
    66: "Freezing Rain: Light and heavy intensity",
    67: "Freezing Rain: Light and heavy intensity",
    71: "Snow fall: Slight, moderate, and heavy intensity",
    73: "Snow fall: Slight, moderate, and heavy intensity",
    75: "Snow fall: Slight, moderate, and heavy intensity",
    77:	"Snow grains",
    80: "Rain showers: Slight, moderate, and violent",
    81: "Rain showers: Slight, moderate, and violent",
    82: "Rain showers: Slight, moderate, and violent",
    85: "Snow showers slight and heavy",
    86: "Snow showers slight and heavy",
    95: "Thunderstorm: Slight or moderate",
    96: "Thunderstorm with slight and heavy hail",
    99: "Thunderstorm with slight and heavy hail"
}

def get_weather_data(coordinates):
    '''
    Fetches weather data from the Open-Meteo API for the given latitude and longitude.

    Args:
        coordinates (tuple): The latitude of the location.

    Returns:
        float: The current temperature in the coordinates you've asked for
    '''
    req = requests.get("https://api.open-meteo.com/v1/forecast?latitude=%f&longitude=%f&current=temperature_2m&current=weather_code" % coordinates)
    j = req.json()
    o = "The current temperature is %f°C with an outlook of '%s'" % (j["current"]["temperature_2m"], WEATHER_CODES[j["current"]["weather_code"]])
    print(o)
    return o


def get_coordinates_from_city(city_name):
    '''
    Fetches the latitude and longitude of a given city name using the Bing Geocoding API.

    Args:
        city_name (str): The name of the city.

    Returns:
        tuple: The latitude and longitude of the city.
    '''

    g = geocoder.bing(city_name, key = os.environ["BINGMAPS"]).json
    print("By '%s' I am assuming you mean '%s'" % (city_name, g["address"]))

    # print(g["lat"], g["lng"])
    return g["lat"], g["lng"]

if __name__ == "__main__":
    print(get_weather_data(get_coordinates_from_city("Lincoln")))
