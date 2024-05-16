docker build -t sailbot_event_manageer --build-arg NODE_NAME=sailbot_event_manager .
docker build -t sailbot_autopilot --build-arg NODE_NAME=autopilot .
docker build -t sailbot_telemetry --build-arg NODE_NAME=telemetry .
docker build -t sailbot_mcu --build-arg NODE_NAME=mcu .

docker build -t sailbot_wind_sensor --build-arg NODE_NAME=wind_sensor .
docker build -t sailbot_gps --build-arg NODE_NAME=gps .
docker build -t sailbot_rc --build-arg NODE_NAME=rc .