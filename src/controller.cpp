#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <nano_quad/tunerConfig.h>

double kp[6];
double kd[6];
double ki[6];
double kp_v[6];
double kd_v[6];
double ki_v[6];
double gp[6];

double minOutput;
double maxOutput;

class controller
{

private:
    ros::Publisher vel;
	  ros::Subscriber pos;
    geometry_msgs::Twist new_vel;

//    double kp[6];
//    double kd[6];
//    double ki[6];

    double integratorMin;
    double integratorMax;
    double integral[6];
    double integralangz;

    double previousError[6];
    double error[6];
//    double gp[6];
    double cp[6];
    double output[6];
    int c;

	  ros::Time previousTime;
    ros::Time previousTimeangz;

	  tf::TransformListener m_listener;
  	tf::StampedTransform transform;

// For update2
//    double kp_v[6];
//    double kd_v[6];
//    double ki_v[6];
    double error_v[6];
    double output_p[6];
    double previousError_v[6];
    double cv[6];
    double gv[6];
    double integral_v[6];
    double previous_output[6];


public:
	controller(const ros::NodeHandle& nh):
				m_listener(),
				integratorMin(0.01),
				integratorMax(0.01)

	{
		ros::NodeHandle n;

/*    ros::param::get("/nano_quad/kix",ki[0]);
    ros::param::get("/nano_quad/kiy",ki[1]);
    ros::param::get("/nano_quad/kiz",ki[2]);
    ros::param::get("/nano_quad/kiroll",ki[3]);
    ros::param::get("/nano_quad/kipitch",ki[4]);
    ros::param::get("/nano_quad/kiyaw",ki[5]);

    ros::param::get("/nano_quad/kdx",kd[0]);
    ros::param::get("/nano_quad/kdy",kd[1]);
    ros::param::get("/nano_quad/kdz",kd[2]);
    ros::param::get("/nano_quad/kdroll",kd[3]);
    ros::param::get("/nano_quad/kdpitch",kd[4]);
    ros::param::get("/nano_quad/kdyaw",kd[5]);

    ros::param::get("/nano_quad/kpx",kp[0]);
    ros::param::get("/nano_quad/kpy",kp[1]);
    ros::param::get("/nano_quad/kpz",kp[2]);
    ros::param::get("/nano_quad/kproll",kp[3]);
    ros::param::get("/nano_quad/kppitch",kp[4]);
    ros::param::get("/nano_quad/kpyaw",kp[5]);

    ros::param::get("/nano_quad/gx",gp[0]);
    ros::param::get("/nano_quad/gy",gp[1]);
    ros::param::get("/nano_quad/gz",gp[2]);
    ros::param::get("/nano_quad/groll",gp[3]);
    ros::param::get("/nano_quad/gpitch",gp[4]);
    ros::param::get("/nano_quad/gyaw",gp[5]);
*/
    for (c=0;c<6;c++)
    {
      cv[c]=0;
    }

//    dynamic_reconfigure::Server<nano_quad::tunerConfig> server;
//    dynamic_reconfigure::Server<nano_quad::tunerConfig>::CallbackType f;
//    f = boost::bind(&controller::callback, this, _1, _2);
//    server.setCallback(f);
//    ROS_INFO("Dynamic Reconfigure Server Started");

    vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
		pos = n.subscribe("/vicon/bebop2/bebop2",1,&controller::pid_ctrl,this);

	}
/*
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

    std::cout<<config.kpx<<std::endl;
    std::cout<<std::endl;

    kp_v[0] = config.kp_vx;
    ki_v[0] = config.ki_vx;
    kd_v[0] = config.kd_vx;

    kp_v[1] = config.kp_vy;
    ki_v[1] = config.ki_vy;
    kd_v[1] = config.kd_vy;

    kp_v[2] = config.kp_vz;
    ki_v[2] = config.ki_vz;
    kd_v[2] = config.kd_vz;

  }
*/

	void randomcallback(const geometry_msgs::Twist& old_vel){}

	void pid_ctrl(const geometry_msgs::TransformStamped& pos)
	{
		m_listener.lookupTransform("/world","/vicon/bebop2/bebop2", ros::Time(0), transform);

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
    new_vel.linear.x = cos(-yaw)*output[0]-sin(-yaw)*output[1];
    new_vel.linear.y = sin(-yaw)*output[0]+cos(-yaw)*output[1];
    new_vel.linear.z = output[2];
    new_vel.angular.x = output[3];
    new_vel.angular.y = output[4];
    new_vel.angular.z = output[5];

	new_vel.linear.x=std::max(minOutput,(std::min(new_vel.linear.x,maxOutput)));
	new_vel.linear.y=std::max(minOutput,(std::min(new_vel.linear.y,maxOutput)));
	new_vel.linear.z=std::max(minOutput,(std::min(new_vel.linear.z,maxOutput)));
    new_vel.angular.x=std::max(minOutput,(std::min(new_vel.angular.x,maxOutput)));
    new_vel.angular.y=std::max(minOutput,(std::min(new_vel.angular.y,maxOutput)));
    new_vel.angular.z=std::max(minOutput,(std::min(new_vel.angular.z,maxOutput)));
/*
    std::cout<<new_vel.linear.x<<std::endl;
    std::cout<<new_vel.linear.y<<std::endl;
    std::cout<<new_vel.linear.z<<std::endl;
  	std::cout<<new_vel.angular.x<<std::endl;
  	std::cout<<new_vel.angular.y<<std::endl;
    std::cout<<new_vel.angular.z<<std::endl;

		new_vel.linear.x=0.0;
		new_vel.linear.y=0.0;
		new_vel.linear.z=0.0;
    std::cout<<std::endl;
	  new_vel.angular.x=0;
    new_vel.angular.y=0;
    new_vel.angular.z=0;
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
        output[c]=std::max(minOutput,(std::min(output[c],maxOutput)));
      }
      previousTime = time;
  }

//New update function that uses P controller for distance input to PID controller for velocity
  void update2()
  {
    ros::Time time = ros::Time::now();
    double dt = time.toSec() - previousTime.toSec();

  // PID controller with distance from goal as inout; tuned to output goal velocity
  // Output is simply p+i+d, not prev distance+p+i+d
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
          cv[c] = (error[c]-previousError[c]) / dt;
      }
      double i = ki[c] * integral[c];
      //gv[c] = (p + d + i);
      gv[c] = p;
      previousError[c] = error[c];
      gv[c]=std::max(minOutput,(std::min(gv[c],maxOutput)));

  // Using P input of position as input for velocity
      error_v[c] = gv[c] - cv[c];
      integral_v[c] += error_v[c] * dt;
      integral_v[c] = std::max(std::min(integral_v[c], integratorMax), integratorMin);
      p = kp_v[c] * error_v[c];
      if (dt > 0)
      {
          d = kd[c] * (error_v[c] - previousError_v[c]) / dt;
      }
      i = ki_v[c] * integral_v[c];
      output[c] = gv[c]+(p + d + i)*0.1;
      previousError_v[c] = error_v[c];
      output[c] = std::max(minOutput,(std::min(output[c],maxOutput)));
      previous_output[c] = output[c];
    }
    previousTime = time;

  }

};


void callback(nano_quad::tunerConfig &config, uint32_t level)
{
  ROS_INFO("INCALLBACK");
  kp[0] = config.kpxy;
  ki[0] = config.kixy;
  kd[0] = config.kdxy;

  kp[1] = config.kpxy;
  ki[1] = config.kixy;
  kd[1] = config.kdxy;

  kp[2] = config.kpz;
  ki[2] = config.kiz;
  kd[2] = config.kdz;

  kp[3] = config.kprp;
  ki[3] = config.kirp;
  kd[3] = config.kdrp;

  kp[4] = config.kdrp;
  ki[4] = config.kdrp;
  kd[4] = config.kdrp;

  kp[5] = config.kpyaw;
  ki[5] = config.kiyaw;
  kd[5] = config.kdyaw;
/*
  kp_v[0] = config.kp_vxy;
  ki_v[0] = config.ki_vxy;
  kd_v[0] = config.kd_vxy;

  kp_v[1] = config.kp_vxy;
  ki_v[1] = config.ki_vxy;
  kd_v[1] = config.kd_vxy;

  kp_v[2] = config.kp_vz;
  ki_v[2] = config.ki_vz;
  kd_v[2] = config.kd_vz;

  kp_v[3] = config.kp_vrp;
  ki_v[3] = config.ki_vrp;
  kd_v[3] = config.kd_vrp;

  kp_v[4] = config.kp_vrp;
  ki_v[4] = config.ki_vrp;
  kd_v[4] = config.kd_vrp;

  kp_v[5] = config.kp_vyaw;
  ki_v[5] = config.ki_vyaw;
  kd_v[5] = config.kd_vyaw;
*/
  gp[0] = config.gx;
  gp[1] = config.gy;
  gp[2] = config.gz;

  minOutput = config.minOutput;
  maxOutput = config.maxOutput;

//  std::cout<<config.kpx<<std::endl;
//  std::cout<<std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh("~");
    ros::Rate r(int(50));
	  controller ctrl(nh);
    dynamic_reconfigure::Server<nano_quad::tunerConfig> server;
    dynamic_reconfigure::Server<nano_quad::tunerConfig>::CallbackType f;
    f = boost::bind(&callback,  _1, _2 );
    server.setCallback(f);

  	ros::spin();
	return 0;
}
