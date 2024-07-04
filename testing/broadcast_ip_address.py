import requests
import time
import os


TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'


def main():
    while True:
        ip_address = str(os.popen('hostname -I').read()).split(" ")[0]
        requests.post(TELEMETRY_SERVER_URL + "api/wp/one", json={"w": {"ip": ip_address}}).text
        time.sleep(0.2)


if __name__ == "__main__":
    main()