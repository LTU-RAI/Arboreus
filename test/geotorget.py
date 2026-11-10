import yaml, requests

from requests.auth import HTTPBasicAuth

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

if __name__ == "__main__":

    params = {
        "srid": 3006,
        "e": 828750.5530509967,
        "n": 7294642.407262497,
    }
    
    response = requests.get(
        config["LANTMATERIET_URL"],
        params=params,
        headers={"Accept": "application/json"},
        auth=HTTPBasicAuth(config["LANTMATERIET_USERNAME"], config["LANTMATERIET_PASSWORD"]),
        timeout=30,
    )

    response.raise_for_status()
    data = response.json()

    height = data["geometry"]["coordinates"][2]
    nodata = data["properties"]["nodatavalue"]

    if height == nodata:  print("No height data available")
    else:                 print(f"Height: {height:.2f} m")