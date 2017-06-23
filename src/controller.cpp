#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <nano_quad/tunerConfig.h>

class controller
{

private:
    ros::Publisher vel;
	  ros::Subscriber pos;
    geometry_msgs::Twist new_vel;

    double kp[6];
    double kd[6];
    double ki[6];
    double minOutput;
    double maxOutput;
    double integratorMin;
    double integratorMax;
    double integral[6];
    double integralangz;

    double previousError[6];
    double error[6];
    double gp[6];
    double cp[6];
    double output[6];
    int c;
	  ros::Time previousTime;
    ros::Time previousTimeangz;

	  tf::TransformListener m_listener;
  	tf::StampedTransform transform;

public:
	controller(const ros::NodeHandle& nh):
				m_listener(),
				maxOutput(0.5),
				minOutput(-0.5),
				integratorMin(0.01),
				integratorMax(0.01)
	{
		ros::NodeHandle n;
    n.getParam("/nano_quad/kix",ki[0]);
    n.getParam("/nano_quad/kiy",ki[1]);
    n.getParam("/nano_quad/kiz",ki[2]);
    n.getParam("/nano_quad/kiroll",ki[3]);
    n.getParam("/nano_quad/kipitch",ki[4]);
    n.getParam("/nano_quad/kiyaw",ki[5]);

    n.getParam("/nano_quad/kdx",kd[0]);
    n.getParam("/nano_quad/kdy",kd[1]);
    n.getParam("/nano_quad/kdz",kd[2]);
    n.getParam("/nano_quad/kdroll",kd[3]);
    n.getParam("/nano_quad/kdpitch",kd[4]);
    n.getParam("/nano_quad/kdyaw",kd[5]);

    n.getParam("/nano_quad/kpx",kp[0]);
    n.getParam("/nano_quad/kpy",kp[1]);
    n.getParam("/nano_quad/kpz",kp[2]);
    n.getParam("/nano_quad/kproll",kp[3]);
    n.getParam("/nano_quad/kppitch",kp[4]);
    n.getParam("/nano_quad/kpyaw",kp[5]);

    n.getParam("/nano_quad/gx",gp[0]);
    n.getParam("/nano_quad/gy",gp[1]);
    n.getParam("/nano_quad/gz",gp[2]);
    n.getParam("/nano_quad/groll",gp[3]);
    n.getParam("/nano_quad/gpitch",gp[4]);
    n.getParam("/nano_quad/gyaw",gp[5]);

    dynamic_reconfigure::Server<nano_quad::tunerConfig> server;
    dynamic_reconfigure::Server<nano_quad::tunerConfig>::CallbackType f;
    f = boost::bind(&controller::callback, this, _1, _2);
    server.setCallback(f);
    ROS_INFO("Dynamic Reconfigure Server Started");

    vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
		pos = n.subscribe("/vicon/bebop/bebop",1,&controller::pid_ctrl,this);
	}

  void callback(nano_quad::tunerConfig &config, uint32_t level)
  {
    ROS_INFO("INCALLBACK");
  	kp[0] = config.kpx;
  	ki[0] = config.kix;
  	kd[0] = config.kdx;

    kp[1] = config.kpy;
  	ki[1] = config.kiy;
  	kd[1] = config.kdy;

    kp[2] = config.kpz;
  	ki[2] = config.kiz;
  	kd[2] = config.kdz;
  }


	void randomcallback(const geometry_msgs::Twist& old_vel){}

	void pid_ctrl(const geometry_msgs::TransformStamped& pos)
	{
		m_listener.lookupTransform("/world","/vicon/bebop/bebop", ros::Time(0), transform);

    tfScalar roll, pitch, yaw;
    tf::Matrix3x3(
        tf::Quaternion(
            transform.getRotation().x(),
            transform.getRotation().y(),
            transform.getRotation().z(),
            transform.getRotation().w()
        )).getRPY(roll, pitch, yaw);

    cp[0]=transform.getOrigin().x();
    cp[1]=transform.getOrigin().y();
    cp[2]=transform.getOrigin().z();
    cp[3]=roll;
    cp[4]=pitch;
    cp[5]=yaw;

		update();
/*
    std::cout<<output[0]<<std::endl;
    std::cout<<output[1]<<std::endl;
    std::cout<<output[2]<<std::endl;
    std::cout<<std::endl;

    std::cout<<roll<<std::endl;
    std::cout<<pitch<<std::endl;
    std::cout<<yaw<<std::endl;
    std::cout<<std::endl;
*/
    new_vel.linear.x = cos(yaw)*output[0]-sin(yaw)*output[1];
    new_vel.linear.y = sin(yaw)*output[0]+cos(yaw)*output[1];
    new_vel.linear.z = output[2];
    new_vel.angular.x = output[3];
    new_vel.angular.y = output[4];
    new_vel.angular.z = output[5];

		new_vel.linear.x=std::max(-0.05,(std::min(new_vel.linear.x,0.05)));
		new_vel.linear.y=std::max(-0.05,(std::min(new_vel.linear.y,0.05)));
		new_vel.linear.z=std::max(-0.05,(std::min(new_vel.linear.z,0.05)));
    new_vel.angular.x=std::max(-0.05,(std::min(new_vel.angular.x,0.05)));
    new_vel.angular.y=std::max(-0.05,(std::min(new_vel.angular.y,0.05)));
    new_vel.angular.z=std::max(-0.05,(std::min(new_vel.angular.z,0.05)));

/*
    std::cout<<new_vel.linear.x<<std::endl;
    std::cout<<new_vel.linear.y<<std::endl;
    std::cout<<new_vel.linear.z<<std::endl;
  	std::cout<<new_vel.angular.x<<std::endl;
  	std::cout<<new_vel.angular.y<<std::endl;
    std::cout<<new_vel.angular.z<<std::endl;
    std::cout<<std::endl;

//		new_vel.linear.x=0.0;
//		new_vel.linear.y=0.0;
//		new_vel.linear.z=0.0;
//	  new_vel.angular.x=0;
//    new_vel.angular.y=0;
//    new_vel.angular.z=0;
*/
		vel.publish(new_vel);

	}

  void update()
  {
      ros::Time time = ros::Time::now();
      double dt = time.toSec() - previousTime.toSec();
      for (c=0;c<6;c++)
      {

        error[c] = gp[c] - cp[c];
        integral[c] += error[c] * dt;
        integral[c] = std::max(std::min(integral[c], integratorMax), integratorMin);
        double p = kp[c] * error[c];
        double d = 0;
        if (dt > 0)
        {
            d = kd[c] * (error[c] - previousError[c]) / dt;
        }
        double i = ki[c] * integral[c];
        output[c] = (p + d + i);
        previousError[c] = error[c];
      }
      previousTime = time;
  }

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh("~");
    ros::Rate r(int(50));
	  controller ctrl(nh);
  	ros::spin();
	return 0;
}
