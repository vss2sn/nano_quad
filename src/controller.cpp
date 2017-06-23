#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


class controller
{

private:
    ros::Publisher vel;
	  ros::Subscriber pos;
    geometry_msgs::Twist new_vel;
    geometry_msgs::Twist new_abs_vel;

    double kp[3];
    double kd[3];
    double ki[3];
    double minOutput;
    double maxOutput;
    double integratorMin;
    double integratorMax;
    double integral[3];
    double integralangz;

    double previousError[3];
    double previousErrorangz;
    double error[3];
    double gp[3];
    double agz;
    double cp[3];
    double output[3];
    int c;
	  ros::Time previousTime;
    ros::Time previousTimeangz;

	tf::TransformListener m_listener;
  	tf::StampedTransform transform;

public:
	controller(const ros::NodeHandle& nh):
		    	agz(0.0),
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
    n.getParam("/nano_quad/kdx",kd[0]);
    n.getParam("/nano_quad/kdy",kd[1]);
    n.getParam("/nano_quad/kdz",kd[2]);
    n.getParam("/nano_quad/kpx",kp[0]);
    n.getParam("/nano_quad/kpy",kp[1]);
    n.getParam("/nano_quad/kpz",kp[2]);
    n.getParam("/nano_quad/gx",gp[0]);
    n.getParam("/nano_quad/gy",gp[1]);
    n.getParam("/nano_quad/gz",gp[2]);

    vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
		pos = n.subscribe("/vicon/bebop/bebop",1,&controller::pid_ctrl,this);

	}

	void randomcallback(const geometry_msgs::Twist& old_vel){}

	void pid_ctrl(const geometry_msgs::TransformStamped& pos)
	{

    //std::cout<<ki[1]<<std::endl;
		m_listener.lookupTransform("/world","/vicon/bebop/bebop", ros::Time(0), transform);
    cp[0]=transform.getOrigin().x();
    cp[1]=transform.getOrigin().y();
    cp[2]=transform.getOrigin().z();
		update();

//    std::cout<<output[0]<<std::endl;
//    std::cout<<output[1]<<std::endl;
//    std::cout<<output[2]<<std::endl;
//    std::cout<<std::endl;
    std::cout<<transform.getRotation().z()<<std::endl;
    new_vel.linear.x = cos(M_PI*transform.getRotation().z())*output[0]-sin(M_PI*transform.getRotation().z())*output[1];
    new_vel.linear.y = sin(M_PI*transform.getRotation().z())*output[0]+cos(M_PI*transform.getRotation().z())*output[1];

		new_vel.linear.x=std::max(-0.05,(std::min(new_vel.linear.x,0.05)));
		new_vel.linear.y=std::max(-0.05,(std::min(new_vel.linear.y,0.05)));
		new_vel.linear.z=std::max(-0.05,(std::min(output[2],0.05)));
//		new_vel.angular.z=std::max(-0.7,(std::min(new_vel.angular.z,0.7)));

//  	std::cout<<new_vel.angular.x<<std::endl;
//  	std::cout<<new_vel.angular.y<<std::endl;
//    std::cout<<new_vel.angular.z<<std::endl;
//    std::cout<<std::endl;

		new_vel.linear.x=0.0;
		new_vel.linear.y=0.0;
		new_vel.linear.z=0.0;
	  new_vel.angular.z=0;

		vel.publish(new_vel);

	}

  void update()
  {
      ros::Time time = ros::Time::now();
      double dt = time.toSec() - previousTime.toSec();
      for (c=0;c<3;c++)
      {
    //    std::cout<<"Goal point:"<<std::endl;
    //    std::cout<<gp[c]<<std::endl;
    //    std::cout<<"Current point:"<<std::endl;
    //    std::cout<<cp[c]<<std::endl;

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
        output[c] = (p + d + i)*1;
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
