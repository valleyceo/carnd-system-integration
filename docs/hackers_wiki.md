# Hacker's Wiki

## Resources

### Tensorflow Pipeline

[Traffic Light Detection and Classification with TensorFlow Object Detection API](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)

[Tensorflow Object Detection(official)](https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb)

Step by Step TensorFlow Object Detection API Tutorial: [part 1](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e) [part 2](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-2-converting-dataset-to-tfrecord-47f24be9248d)


## ROS Environemnt

### VM Setup guide
#### Method 1. Docker
Follow the guide [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77). 

Easier way from [Bydavy](https://github.com/bydavy/CarND-Capstone-Docker).

1. Install Docker.  
2. Go to your project directory.  
3. >> wget https://raw.githubusercontent.com/bydavy/CarND-Capstone-Docker/master/utils/run.sh && chmod u+x run.sh  
4. >> ./run.sh
	- Running this on each new terminal will link to corresponding container
5. >> pip install -r requirements.txt.

#### Method 2. VirtualBox
Follow the guide [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/1f6c617c-c8f2-4b44-9906-d192ba7ff924)

#### Running ROS

Adding Alias on bash profile:
```
echo "rosm=catkin_make && source devel/setup.bash" >> ~/.bashrc  
```
```
echo "rosl=roslaunch launch/styx.launch" >> ~/.bashrc
```  
```
(..\ros\) >> rosm && rosl
```

### Debugging
#### Checking messages
```
rosnode list (running nodes)
rostopic list (running topics)
rostopic info /final waypoints (checks specific topic)
rostopic echo /topic (view message real-time)
rosmsg show /msg_type
rosmsg info /msg_type
rosed [package_name] [filename] (view and edit)
```

#### Console debugging

```
# View all log messages
rqt_console

# View image topic
rosrun image_view image_view image:=/image_color (rostopic name)
```

Checking out rosbag: [link](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/3251f513-2f82-4d5d-88b6-9d646bbd9101)

## GitHub Environemnt

### Commit
```
# get latest repository
git fetch
git merge

# pushing
git add .
git commit -m "commit message"
git push
```

### branch control
```
# new branch
git checkout <new_branch> (create a new branch)
git push <remote_name> <branch_name>
git branch -a (list all branch)

# moving to branches without pushing the current repo
git stash (or git stash save)
git stash apply stash@{0} (or just git stash apply to load the latest stash)
git stash list (get all list of stash)
git stash pop (remove the stash)
```

### Resolving Conflict
```
# check difference
git diff
```
