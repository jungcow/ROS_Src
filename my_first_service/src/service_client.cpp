#include <cstdlib>
#include "my_first_service/MyFirstSrv.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_client");
	if (argc != 3)
	{
		ROS_INFO("usage: add_two_reals_client X Y");
		return 1;
	}
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<my_first_service::MyFirstSrv>("calculate_two_reals");
	my_first_service::MyFirstSrv srv;

	srv.request.a = atof(argv[1]);
	srv.request.b = atof(argv[2]);
	if(client.call(srv))
	{
		ROS_INFO("Sum: %lf", (double)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_reals");
		return 1;
	}
	return (0);
}
