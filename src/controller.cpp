#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


class controller
{

private:
    ros::Publisher vel;
	  ros::Subscriber pos;
    //ros::Subscriber old_vel;
    geometry_msgs::Twist new_vel;

    double kp;
    double kd;
    double ki;
    double minOutput;
    double maxOutput;
    double integratorMin;
    double integratorMax;
    double integralx;
    double integraly;
    double integralz;
    double previousErrorx;
    double previousErrory;
    double previousErrorz;
    double gx;
  	double gy;
  	double gz;
    double agz;
    ros::Time previousTimex;
    ros::Time previousTimey;
    ros::Time previousTimez;
  	tf::TransformListener m_listener;
  	tf::StampedTransform transform;

public:
	controller(const ros::NodeHandle& nh):
				kp(0.2),
				kd(0.1),
				ki(0.01),
				gx(0.0),
				gy(0.0),
				gz(0.5),
        agz(0.0),
				m_listener(),
				maxOutput(0.5),
				minOutput(-0.5),
				integratorMin(0.01),
				integratorMax(0.01)
	{
		ros::NodeHandle n;
		vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
		pos = n.subscribe("/vicon/bebop/bebop",1,&controller::pid_ctrl,this);
    //old_vel = n.subscribe("/bebop/cmd_vel",1,&controller::randomcallback,this);
	}

  void randomcallback(const geometry_msgs::Twist& old_vel){}

	void pid_ctrl(const geometry_msgs::TransformStamped& pos)
	{

		m_listener.lookupTransform("/world","/vicon/bebop/bebop", ros::Time(0), transform);
		new_vel.linear.x = updatex(transform.getOrigin().x(),gx);
		new_vel.linear.y = updatey(transform.getOrigin().y(),gy);
		new_vel.linear.z = updatez(transform.getOrigin().z(),gz);
    new_vel.angular.z = updateangz(transform.getRotation().z(),agz);
    //std::cout<<transform.getRotation().z()<<std::endl;
		new_vel.linear.x=std::max(-0.2,(std::min(new_vel.linear.x,0.2)));
		new_vel.linear.y=std::max(-0.2,(std::min(new_vel.linear.y,0.2)));
		new_vel.linear.z=std::max(-0.2,(std::min(new_vel.linear.z,0.2)));
    new_vel.angular.z=std::max(-0.01,(std::min(new_vel.angular.z,0.01)));
		vel.publish(new_vel);
	}

//  X
    double updatex(double value, double targetValue)
    {
        ros::Time time = ros::Time::now();
        double dt = time.toSec() - previousTimex.toSec();
        double error = targetValue - value;

        integralx += error * dt;
        integralx = std::max(std::min(integralx, integratorMax), integratorMin);
        double p = kp * error;
        double d = 0;
        if (dt > 0)
        {
            d = kd * (error - previousErrorx) / dt;
        }
        double i = ki * integralx;
        double output = p + d + i;
        previousErrorx = error;
        previousTimex = time;
        return std::max(std::min(output, maxOutput), minOutput);
    }

//Y
    double updatey(double value, double targetValue)
    {
        ros::Time time = ros::Time::now();
        double dt = time.toSec() - previousTimey.toSec();
        double error = targetValue - value;

        integraly += error * dt;
        integraly = std::max(std::min(integraly, integratorMax), integratorMin);
        double p = kp * error;
        double d = 0;
        if (dt > 0)
        {
            d = kd * (error - previousErrory) / dt;
        }
        double i = ki * integraly;
        double output = p + d + i;
        previousErrory = error;
        previousTimey = time;
        return std::max(std::min(output, maxOutput), minOutput);
    }

//Z
    double updatez(double value, double targetValue)
    {
        ros::Time time = ros::Time::now();
        double dt = time.toSec() - previousTimez.toSec();
        double error = targetValue - value;

        integralz += error * dt;
        integralz = std::max(std::min(integralz, integratorMax), integratorMin);
        double p = kp * error;
        double d = 0;
        if (dt > 0)
        {
            d = kd * (error - previousErrorz) / dt;
        }
        double i = ki * integralz;
        double output = p + d + i;
        previousErrorz = error;
        previousTimez = time;
        return std::max(std::min(output, maxOutput), minOutput);
    }

    double updateangz(double value, double targetValue)
    {
        ros::Time time = ros::Time::now();
        double dt = time.toSec() - previousTimez.toSec();
        double error = targetValue - value;

        integralz += error * dt;
        integralz = std::max(std::min(integralz, integratorMax), integratorMin);
        double p = kp * error;
        double d = 0;
        if (dt > 0)
        {
            d = kd * (error - previousErrorz) / dt;
        }
        double i = ki * integralz;
        double output =0.1*(p + d + i);
        previousErrorz = error;
        previousTimez = time;
        return std::max(std::min(output, maxOutput), minOutput);
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
