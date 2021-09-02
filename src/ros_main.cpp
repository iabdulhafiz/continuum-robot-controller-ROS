
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>


// includes
#include <ctcr_model.h>
#include <mainloop.h>
#include <tdcr_model.h>
#include <visualizer.h>

// stl
#include <array>
#include <cmath>
#include <iostream>

// vtk
#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>

// Eigen
#include "Eigen/Dense"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%






// Choose which scenario to set up and set parameters accordingly
  const char *scen;
  if (argc > 1)
    scen = argv[1];
  else
    scen = "a0";

  int assignment = 4;
  if (strcmp(scen, "a4") == 0) {
    assignment = 4;
  }

  // Create Visualizer
  Visualizer vis;
  vis.initScene(assignment);

  // Create TDCR
  std::array<double, 2> length;
  length[0] = 0.1;
  length[1] = 0.1;
  int n_disks = 8;
  std::array<double, 2> pradius_disks;
  pradius_disks[0] = 0.006;
  pradius_disks[1] = 0.005;

  Eigen::Matrix4d base_frame = Eigen::Matrix4d::Identity();

  double radius_disks = 0.007;
  double height_disks = 0.003;
  double ro = 0.001;

  TDCRModelDVS tdcr_model(length, n_disks, pradius_disks, base_frame);

  if (assignment == 4) {
    std::cout << "Setting up assignment 4..." << std::endl;
    std::cout << "Choose control scenario - (0) TDCR or (1) CTCR: ";

    std::cout << std::endl;

      Eigen::Matrix<double, 2, 1> q;
      q << -0.005, 0.0025 ;
      Eigen::Matrix4d ee_frame;
      Eigen::MatrixXd disk_frames;

      if (tdcr_model.forward_kinematics(ee_frame, disk_frames, q)) {
        vis.drawTDCR(n_disks, pradius_disks, radius_disks, ro, height_disks);
        vis.updateTDCR(disk_frames);
      }
  }
  // Turn off warning messages to prevent them from spamming the terminal
  vtkObject::GlobalWarningDisplayOff();

  // Define required variables to set up the simulation
  double timestep = 0.01;

  // Create Main Loop
  MainLoop eventLoop(&vis, &tdcr_model, timestep,
                     assignment);

  // Create Window Interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(vis.getRenderWindow());

  // Set up and start main loop
  renderWindowInteractor->UpdateSize(1200, 700);
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  renderWindowInteractor->Initialize();
  renderWindowInteractor->CreateRepeatingTimer(timestep * 1000.0);
  renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, &eventLoop);
  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, &eventLoop);
  renderWindowInteractor->Start();

  //return 1;








  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
