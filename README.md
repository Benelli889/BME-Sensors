# BME-Sensors
Bosch Sensors BME280 BME680

Autostart: airquality.desktop
/etc/xdg/autostart

BME280:

sudo crontab -e
@reboot bash /home/pi/bme280-python/examples/BME280.sh
