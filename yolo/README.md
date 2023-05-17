Need vision msgs, so please run:
```bash
sudo apt install ros-noetic-vision-msgs
```

The weights for the model are too large to upload to git so please download these inside of yolo_config:

```bash
cd yolo_config
wget https://pjreddie.com/media/files/yolov3.weights
```

Run node using 

```bash
rosrun yolo yolo_detection.py 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

The part behind yolo_detection.py makes sure terminal is not flooded with warnings (tf has a known issue that is non-critical)
