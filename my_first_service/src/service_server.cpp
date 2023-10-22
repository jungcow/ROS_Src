#include "my_first_service/MyFirstSrv.h"
#include "ros/ros.h"

#define PLUS 1
#define MINUS 2
#define MUL 3
#define DIV 4

int g_operator = PLUS;

bool calculate(my_first_service::MyFirstSrv::Request &req,
		my_first_service::MyFirstSrv::Response &res) {
	switch(g_operator)
	{
		case PLUS:
			res.sum = req.a + req.b;
			break;
		case MINUS:
			res.sum = req.a - req.b;
			break;
		case MUL:
			res.sum = req.a * req.b;
			break;
		case DIV:
			if (req.b == 0)
				res.sum = 0;
			else
				res.sum = req.a / req.b;
			break;
		default:
			res.sum = 0;
			break ;
	}
	ROS_INFO("request: x: %lf, y: %lf", (double)req.a, (double)req.b);
	ROS_INFO("response: [%lf]", (double)res.sum);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_server");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("calculate_two_reals", calculate);
	ROS_INFO("Ready to calculate two reals");
	ros::Rate r(10);
	while (ros::ok())
	{
		nh.param("calculation_method", g_operator, PLUS);
		ros::spinOnce();
		r.sleep();
	}
	return (0);
}
