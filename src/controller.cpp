// 04.08.17

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <nano_quad/tunerConfig.h>
#include <cmath>

// Global Variables

int flag;
int mcv;

double kps;
double kis;
double kds;

double kp[6];
double kd[6];
double ki[6];

double kpv;
double kiv;
double kdv;
double maggv;

double gp[6];
double gv; //Currently restircted to y direction

double minOutput;
double maxOutput;

class controller
{

private:
    ros::Publisher vel;
    ros::Publisher vel2;
    ros::Subscriber pos;
    geometry_msgs::Twist new_vel;

    static const int nhist = 15;	
    double integratorMin;
    double integratorMax;
    double integral[6];
    double integralv[6];

    double previousError[6];
    double error[6];
    double previousErrorv[6];
    double errorv[6];

    double cp[6];
	double cv[6];
    double prev_cp[6];
    double output[6];
    
	int c;
    int c2;
    int c2_max;	
    
	double pos_history[6][nhist];
	double step_i;
	double prev_sum;
	ros::Time previousTime;

    tf::TransformListener m_listener;
  	tf::StampedTransform transform;

public:
	controller(const ros::NodeHandle& nh):
				m_listener(),
				integratorMin(0.01),
				integratorMax(0.01)

	{
		c2=0;
		prev_sum=0;
		c2_max=nhist;
		ros::NodeHandle n;
		vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1); // get arg from launch file
		pos = n.subscribe("/vicon/bebop2/bebop2",1,&controller::pid_ctrl,this); // get arg from launch file

	}

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
	
		pos_history[2][c2] = cp[2];
		pos_history[1][c2] = cp[1];	
		c2=c2+1;
		
		if(c2==c2_max)
		{
			c2=0;
		}

		double add = 0; // additional input to z 		
		if (flag==1){
			add = add_step();
		}
	
		update(); // Get update input to controller

		new_vel.linear.x = cos(-yaw)*output[0]-sin(-yaw)*output[1];
		new_vel.linear.y = sin(-yaw)*output[0]+cos(-yaw)*output[1];
		new_vel.linear.z = output[2]+add; // Add aditional input
		new_vel.angular.z = output[ 5];
		new_vel.linear.x=std::max(minOutput,(std::min(new_vel.linear.x,maxOutput)));
		new_vel.linear.y=std::max(minOutput,(std::min(new_vel.linear.y,maxOutput)));
		vel.publish(new_vel); //Publish new values

	}

	double add_step()
	{
		ros::Time time = ros::Time::now();
		double slope[nhist];
		double sum = 0;
		double dt = time.toSec() - previousTime.toSec();
		for(int i=c2+1;i<c2_max;i++)
		{
			slope[i] = (pos_history[2][i]-pos_history[2][i-1])/(pos_history[1][i]-pos_history[1][i-1]);
			if((std::abs(pos_history[2][i]-pos_history[2][i-1]))< 0.005 || std::abs(pos_history[1][i]-pos_history[1][i-1])< 0.005){
				slope[i]=0;
			}
			if(isnan(slope[i])){
				slope[i]=0;
			}
			sum=sum+slope[i];
		}
		for(int i=1;i<=c2;i++)
		{
			slope[i] = (pos_history[2][i]-pos_history[2][i-1])/(pos_history[1][i]-pos_history[1][i-1]);
			if((std::abs(pos_history[2][i]-pos_history[2][i-1]))< 0.005 || std::abs(pos_history[1][i]-pos_history[1][i-1])< 0.005){
				slope[i]=0;
			}				
			if(isnan(slope[i])){
				slope[i]=0;
			}
			sum=sum+slope[i];
		}
		//slope ranges from indices [1,39]
		sum=sum/c2_max;
	    if (dt > 0)
	    {
	    	double d = kds*(sum-prev_sum)/dt;
	    }
		double p = std::abs(kps*sum);		
		// Adjust additional input for relative position to goal height as well as direction of motion
		if (cp[2]-gp[2]>0){
			if(cp[2]-prev_cp[2]>0){
				p=p*(-1);
			}
			else{
				p=0;
			}
		}
		else if (cp[2]-gp[2]<0){
			if(cp[2]-prev_cp[2]<0){
			}
			else{
				p=0;
			}

		}
		std::cout<<sum<<std::endl;
		std::cout<<p<<std::endl;
		std::cout<<std::endl;
		step_i+=sum;
		prev_sum=sum;
		double i = kis*step_i;	
		return (p+i);			
	}

	//PID cntroller for position and veocity
	//Currently gives option to maintain constant velocity for a given range
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
				cv[c] = (cp[c]-prev_cp[c]) / dt;
		    }
		    double i = ki[c] * integral[c];
		    output[c] = (p + d + i);
		    previousError[c] = error[c];  
			// maintain constant velocity
			if ((error[c]>=0.1 || error[c]<=-0.1) && c==1 && mcv==1 && cp[c]>-1.3 && cp[c]<1.3)
			{	
				if (error[c]>=0)
				{
					gv=maggv;
				}
				else
				{
					gv=-1*maggv;
				}
				errorv[c]=gv-cv[c];				
				integralv[c] += errorv[c] * dt;
				integralv[c] = std::max(std::min(integral[c], integratorMax), integratorMin);
				double pv = kpv * errorv[c];
				double dv = 0;
				if (dt > 0)
				{
				    dv = kdv * (errorv[c] - previousErrorv[c]) / dt;
				}
				double iv = kiv * integralv[c];
				output[c] = (pv + dv + iv); 
			    previousErrorv[c] = errorv[c]; 
			}
			prev_cp[c]=cp[c];
		}
	previousTime = time;
	}

};

void callback(nano_quad::tunerConfig &config, uint32_t level)
{
	ROS_INFO("Reconfiguring parameters");
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

	gp[0] = config.gx;
	gp[1] = config.gy;
	gp[2] = config.gz;

	kpv=config.kpv;
	kiv=config.kiv;
	kdv=config.kdv;

	maggv=config.maggv; // Magnitude of velocity to be maintained
	mcv = config.mcv; // Implement constant velocity controller
	flag = config.flag; // Use controller in add step function that uses velocity P controller (ie slope of Z)
	minOutput = config.minOutput; // Maximum oputput to roll and pitch for safety
	maxOutput = config.maxOutput; // Minimum oputput to roll and pitch for safety
	kps = config.kps; // kp for slope
	kis = config.kis; // ki for slope	
	kds = config.kds; // ki for slope	
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
