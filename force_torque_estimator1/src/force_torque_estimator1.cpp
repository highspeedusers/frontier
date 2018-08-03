#include <ros/ros.h>
#include <dora_serial/wheel_unit.h>
#include <geometry_msgs/WrenchStamped.h>
#define _USE_MATH_DEFINES
#include <math.h>
//#include <vector>

class ForceTorqueEstimator
{
private:
		ros::Subscriber ang_sub, ft_raw_sub;
		ros::Publisher ft_pub;
		ros::Time current_time, last_time;
		double wheel_rotation;
 		double wheel_angle_rad;
		double initial_wheel_angle_deg;
		double ft_in_wheel[6];
     		double ft_offset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		double vertical_offset;
		const double pi=M_PI;

public:
		ForceTorqueEstimator();
		~ForceTorqueEstimator(){};
		// void loop();
		// double wheel_angle_rad;
		// double ft_in_wheel[6];
		void wheel_rotation_cb(const dora_serial::wheel_unit& wheel_msg);
		void wheel_ft_cb(const geometry_msgs::WrenchStamped ft_raw);
};

ForceTorqueEstimator::ForceTorqueEstimator()
{
		ros::NodeHandle nh;
			ros::NodeHandle nh_private("~");
	if (!nh_private.getParam("initial_angle_deg", initial_wheel_angle_deg))
	{
		ROS_WARN("initial_angle_deg is not defined, trying 0.0");
	    initial_wheel_angle_deg = 0.0;
	}
	wheel_angle_rad = initial_wheel_angle_deg*pi/180;

	if (!nh_private.getParam("wheel_offset", vertical_offset))
	{
		ROS_WARN("wheel_offset is not defined, using 0.0");
		vertical_offset = 0.0;
	}
	
	if(!nh_private.getParam("offset", ft_offset[6]))
	{
			ROS_WARN("offsets are not defined, trying {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}");
	}
	nh_private.param<double>("offset", ft_offset[6]);

		ang_sub = nh.subscribe("/wheel1", 1, &ForceTorqueEstimator::wheel_rotation_cb,this);
		ft_raw_sub = nh.subscribe("/leptrino_force_torque1/force_torque",1, &ForceTorqueEstimator::wheel_ft_cb,this);
		ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("force_torque1",1);

		ros::Rate rate(50);
    		current_time = ros::Time::now();
    		last_time = ros::Time::now();
		while(ros::ok())
		{
				current_time = ros::Time::now();
				double dt = (current_time - last_time).toSec();
				wheel_angle_rad = wheel_angle_rad + wheel_rotation * dt;
				geometry_msgs::WrenchStampedPtr ft_msg(new geometry_msgs::WrenchStamped);
				ft_msg->header.stamp = ros::Time::now();
				ft_msg->wrench.force.x = ft_in_wheel[0]*std::sin(wheel_angle_rad) \
						- ft_in_wheel[1]*std::cos(wheel_angle_rad);
				ft_msg->wrench.force.y = ft_in_wheel[2] * -1;
				ft_msg->wrench.force.z = ft_in_wheel[0]*std::cos(wheel_angle_rad) \
						+ ft_in_wheel[1]*std::sin(wheel_angle_rad) - vertical_offset;
				ft_msg->wrench.torque.x = ft_in_wheel[3]*std::sin(wheel_angle_rad) \
						- ft_in_wheel[4]*std::cos(wheel_angle_rad);
				ft_msg->wrench.torque.y = ft_in_wheel[5]* -1;
				ft_msg->wrench.torque.z = ft_in_wheel[3]*std::cos(wheel_angle_rad) \
						+ ft_in_wheel[4]*std::sin(wheel_angle_rad);


				last_time = current_time;
				ft_pub.publish(ft_msg);
				ros::spinOnce();		
				rate.sleep();				
		} //while
}

void ForceTorqueEstimator::wheel_ft_cb(const geometry_msgs::WrenchStamped ft_raw)
{
		ft_in_wheel[0] = (ft_raw.wrench.force.x) - ft_offset[0];
		ft_in_wheel[1] = (ft_raw.wrench.force.y) - ft_offset[1];
		ft_in_wheel[2] = (ft_raw.wrench.force.z) - ft_offset[2];
		ft_in_wheel[3] = (ft_raw.wrench.torque.x) - ft_offset[3];
		ft_in_wheel[4] = (ft_raw.wrench.torque.y) - ft_offset[4];
		ft_in_wheel[5] = (ft_raw.wrench.torque.z) - ft_offset[5];
}
void ForceTorqueEstimator::wheel_rotation_cb(const dora_serial::wheel_unit& wheel_msg)
{
		wheel_rotation = wheel_msg.wheel_rotation;
}

int main(int argc, char** argv)
{
		ros::init(argc, argv, "force_torque_estimator1");
		ForceTorqueEstimator ft;
		return 0;
}
