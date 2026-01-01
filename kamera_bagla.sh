#!/bin/bash
source /opt/ros/humble/setup.bash

echo "---------------------------------------------"
echo "   KAMERA KÖPRÜSÜ (IMAGE BRIDGE)             "
echo "---------------------------------------------"
echo "Gazebo topicleri taranıyor..."

KAMERA_TOPICLER=$(gz topic -l | grep "image")

if [ -z "$KAMERA_TOPICLER" ]; then
    echo "HATA: Kamera topic'i bulunamadı! Dronlar henüz açılmamış olabilir."
    exit 1
fi

echo "Bulunan Kameralar:"
echo "$KAMERA_TOPICLER"
echo "---------------------------------------------"

# Topicleri argüman olarak ekliyoruz
CMD="ros2 run ros_gz_image image_bridge"
for TOPIC in $KAMERA_TOPICLER; do
    CMD="$CMD $TOPIC"
done

echo "Başlatılıyor..."
eval $CMD
