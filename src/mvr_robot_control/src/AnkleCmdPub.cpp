#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ankle_cmd_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ankle_cmd", 10);
  
  ros::Rate rate(50);
  double t = 0.0;
  while (ros::ok()) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(4);

    // 示例：左脚 pitch 正弦，roll 0；右脚相反
    msg.data[0] = 0.2 * std::sin(t);  // tpL
    msg.data[1] = 0.0;               // trL
    msg.data[2] = -0.2 * std::sin(t);// tpR
    msg.data[3] = 0.0;               // trR

    // msg.data[0] = 0.0;  
    // msg.data[1] = 0.3 * std::sin(t);              
    // msg.data[2] = 0.0;
    // msg.data[3] = -0.3 * std::sin(t);              

    pub.publish(msg);
    ROS_INFO_STREAM_THROTTLE(0.5,
        "[ankle_cmd_pub] TX /ankle_cmd "
        "tpL=" << msg.data[0] << " trL=" << msg.data[1]
        << " | tpR=" << msg.data[2] << " trR=" << msg.data[3]);

    ros::spinOnce();
    rate.sleep();
    t += 0.05;
  }
  return 0;
}


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "ankle_cmd_pub_manual");
//   ros::NodeHandle nh;

//   ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ankle_cmd", 10);

//   ROS_INFO("[ankle_cmd_pub_manual] Ready. Input: tpL trL tpR trR (rad). Type 'q' to quit.");

//   while (ros::ok()) {
//     std::cout << "Enter tpL trL tpR trR (or q): ";

//     std::string first;
//     if (!(std::cin >> first)) {
//       break;  // stdin closed
//     }
//     if (first == "q" || first == "Q") {
//       break;
//     }

//     double tpL = 0.0, trL = 0.0, tpR = 0.0, trR = 0.0;
//     try {
//       tpL = std::stod(first);
//     } catch (...) {
//       std::cerr << "Invalid input. Try again.\n";
//       continue;
//     }

//     if (!(std::cin >> trL >> tpR >> trR)) {
//       std::cerr << "Invalid input format. Need 4 numbers.\n";
//       std::cin.clear();
//       std::string dummy;
//       std::getline(std::cin, dummy);
//       continue;
//     }

//     std_msgs::Float64MultiArray msg;
//     msg.data.resize(4);
//     msg.data[0] = tpL;
//     msg.data[1] = trL;
//     msg.data[2] = tpR;
//     msg.data[3] = trR;

//     pub.publish(msg);
//     ros::spinOnce();  

//     ROS_INFO_STREAM("[ankle_cmd_pub_manual] Published /ankle_cmd: "
//                     "tpL=" << tpL << " trL=" << trL
//                     << " tpR=" << tpR << " trR=" << trR);
//   }

//   ROS_INFO("[ankle_cmd_pub_manual] Exit.");
//   return 0;
// }
