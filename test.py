import tago

MY_DEVICE_TOKEN = '39ddd257-71cd-4ce9-9473-a272f72edfb6'
my_device = tago.Device(MY_DEVICE_TOKEN)

lat = -27.297429
lng = 153.3

data_to_insert = {
        "variable": "Location",
        "value": 0,
        "location": {
            "lat": lat,
            "lng": lng,
        },
    }
result = my_device.insert(data_to_insert)  # With response
if result['status']:
    print(result['result'])
else:
    print(result['message'])