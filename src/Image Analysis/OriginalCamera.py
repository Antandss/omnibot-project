import requests
from PIL import Image
from io import BytesIO

computer = "philon-11.control.lth.se"
usr = "labwebcam"
pwd = "omnibot-camera"
port = 5000
url = "http://" + usr + ":" + pwd + "@" + computer + ":" + str(port)

response = requests.get(url)
img_data = BytesIO(response.content)
img = Image.open(img_data)
img.show()