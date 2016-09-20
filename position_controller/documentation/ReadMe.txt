To launch the controller: $roslaunch position_controller position_controller.launch

In this launch file you can change the precision (error tolerance) of the joint angles.

In the file: "Publish_topic_position_control.txt " , you can find an example of how to command a desired set of joint angles for Baxter arms (the ones calculated by the force controller):

rostopic pub -1 /move_Baxter sensor_msgs/JointState '{ header: {seq: 0, frame_id: left}, name: [left_e0, left_e1, left_s0, left_s1, left_w0, left_w1, left_w2], position: [0, 0, 0, 0, 0, 0]}'

Speed of Motion
The program uses a type of averaging filter to comand the joint angles "qcom" as follows:

At  t = 0,         qcom = (1-alpha)*qgoal + alpha*q(t)
then for  t > 0,   qcom = (1-alpha)*qgoal + alpha*qcom(t-1)

The defalut value of alpha is set to = 0.9950

You can add the following line to the launch file (after the precision parameter line): <param name="filter" value="0.50" />

And set the value of alpha to whatever you want.

Integrating with the Force Controller:
You can easily merge the position controller with the force controller if you want a close loop with the force error (instead of the position error).
