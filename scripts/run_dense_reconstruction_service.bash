SERVICE_TOPIC_NUM=$1

../bin/dense_reconstruction_service -l /stereo_left/image_raw -r /stereo_right/image_raw -c /home/srabiee/ROS_WS/Campus-Jackal/hardware/calibration/PointGreyCalibration/6_Aug_24_18/jpp/pointgrey_calib_6_Aug_24_18.yaml  -w 960 -h 600  -u 800 -v 540 -d 0 -n $SERVICE_TOPIC_NUM

# ../bin/dense_reconstruction_service -l /stereo_left/image_raw -r /stereo_right/image_raw -c /home/srabiee/ROS_WS/Campus-Jackal/hardware/calibration/PointGreyCalibration/6_Aug_24_18/jpp/pointgrey_calib_6_Aug_24_18.yaml  -w 960 -h 600  -u 900 -v 550 -d 0 -n $SERVICE_TOPIC_NUM
