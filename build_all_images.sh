docker build -t sailbot_wind_sensor --build-arg NODE_NAME=wind_sensor .
docker build -t sailbot_sailcode --build-arg NODE_NAME=sailcode .
docker build -t sailbot_telemetry --build-arg NODE_NAME=telemetry .
docker build -t sailbot_gps --build-arg NODE_NAME=gps .
