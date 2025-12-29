# Reflection & Challenges

## Primary Challenges:

- Google Mediapipe C++ is poorly documented
- Google Mediapipe C++ requires bazel to launch
- Creating a ros2 package containing both python and C++ nodes
- Luanching MoveIt2 with the ros2_kortex API

## Reflection

The original vision for this project was to have a laptop webcam detect a
person's face and use that feedback to move the arm in the direction the face
was moving. Also, since a large part of this project was foraying into C++, I
wanted to use Mediapipe with C++. all of these goals faced unanticipated
challenges.

The biggest problem I faced was getting Mediapipe C++ to work properly. The
first setback I faced was trying to find any documentation regarding the C++
side of the API. In previous project, I've used Mediapipe's python interface
and it was relatively smooth sailing (other than some dated documentation). I
exepcted similar documentation on the C++ side as well, considering that it was
originally written in C++. Instead I found one official "Hello World!" exmample
script and nothing else.

There were, however, a few other tutorials made by individuals who had realized
this documentation gap. However, I quickly learned that to build and use media-
pipe I would have to use bazel, not CMake. This posed a significant linking
problem. Huge shoutout to Mitya Shabat (see appendix for links) who was able
to link Mediapipe using CMake and had documented his process. To link using
CMake is an extremely involved process which never properly worked for me. With
dealines looming, I was forced to pivot to the Mediapipe python API.

Using the python API meant that I needed to have a CPP node and a python node.
This was quickly accomplished using a Shebang and some CMake options. I also
quickly wrote a script to test mediapipe in isolation from ROS, and got frame
rates of 30fps. However, after integrating into a ROS2 Node, I was getting
only 3-4 detect fps. The only reason I could think for the significant change
was that ROS2 was serializing the asynchronous detection calls made by
Mediapipe, effectively blocking the process until detection completed. I first
attempted to use ROS2's multithreadedexecutor but that seemed to only work on
internal ROS2 callbacks. On the second attempt I tried to use python's threading
library to separately thread the Mediapipe function. This had no noticable
effect on the framerate.

At that point, I was using the pose_landmarker lite model from google
which creates a skeleton across the whole body of a human, which would have
allowed for collision checking with a human's body - I want to realize this
project physically after all. In light of the bad performance I decided to try
a different model: the face_landmarker. I thought this might resolve the
issues because it was smaller (256x256 vs 224x224x3). This finally had the
desired effect and allowed the node to run quickly. I was ready to move onto
the actual movement.

Before working on the Mediapipe component, I had done a little bit of work
trying to establish a direct connection to the arm using the ActionClass object.
This works by creating an object instance and then filling in predefined
variables (eg. joint angles) within. However, I quickly found that this method,
in practice, works best when using a physical robot as I couldn't properly
establish a connection in the simulator. Therefore, I pivoted to using a ROS2
topic to publish joint angles, which proved much simpler than the ActionClass
object. At least initially.

After quickly getting the simulator arm moving by ROS topic, I tried to find a
way to control the arm in task space - not joint space. From reading the kortex
api instructions there were a few code snippets for using MoveIt2 to launch the
arm. I _should_ have then been able to use a geometry_msgs Pose message to
control the arm in task space.

At this point there was only a few days left in the project and I could only
control the arm in joint space. I saw two ways forward: I either defined my own
inverse kinematics for the arm to control it in task space or I try to dive into
and learn MoveIt. I had already tried to define kinematics on a Kinova arm and
knew that it was prone to math bugs and very difficult to solve analytically
(since Kinova's hardware geometry doesn't allow for the simple spherical wrist
abstraction at the 5th joint). Additionally, since MoveIt was preconfigured, and
the people I spoke to said MoveIt configuration was the worst part, the answer
was obvious - I would try and use MoveIt.

I spent a frustratingly long time trying to get MoveIt to work. Whenever I tried
to launch the sim using the predefined launch commands I got an 'extra parameter
passed' error. Mentioning that the 'isaac_param' was unknown. I quickly found
that the 'isaac_param' was likely a result of some integration with Nvidia's
Isaac Sim. However, as far as I knew, the isaac sim wasn't installed on my
system and shouldn't be automatically passing parameters into launch files. This
error proved to be the stopping point for this project. I looked through all the
source code pertaining to the launch of Moveit (launch files, associated launch
files, etc.) but couldn't find isaac_param anywhere. I even tried modifying the
launch file to accept the parameter and then do nothing with it yet I still got
the error.

All told this project was a huge learning experience and, despite not reaching
every goal, I am happy with the result. I was able to integrate machine vision
with ROS based robot control. I learned ROS in C++ and have grown comfrotable
integrating with predefined packages to develop my own software. I think that,
considering the amount of time I was able to dedicate to this project, I
achieved and learned much.

## Future

Next steps on this project are getting some sort of task space movement working.
Also, some creature comfort features (like launch file) would be nice. Following
that I plan on exploring sim side camera implementation so I can test person
following through a person model.

## Skill Mastery

I believe I've got a beginner/intermediate mastery of ROS2 on both the python
and C++ side. I've also gained a lot of fringe experience that culminates into a
better system-level understanding of how middleware works and how to debug
errors.
