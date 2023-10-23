
#include <actionlib/server/simple_action_server.h>

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "sdn/GpsInfo.h"
#include "sdn/NewConn.h"
#include "sdn/StateInfo.h"
#include "sdn/VehicleCommandAction.h"

using namespace std;

class Vehicle
{
public:
	enum State
	{
		VS_NEW,
		VS_READYTO_COMMAND,
		VS_COMMAND,
		VS_REQ_NEXT_HOP,
		VS_DEL_PREV_HOP
	};

private:
	ros::NodeHandle nh_;
	// ros::Publisher vstate_pub_;
	ros::ServiceClient vstate_client_;
	ros::ServiceClient connection_client_;
	ros::Rate loop_rate_ = 1;
	int currhop_;
	int prevhop_;
	int nexthop_ = 1;
	int destination_;
	int vid_;
	bool action_result_ = false;
	State vstate_ = VS_NEW;

public:
	Vehicle(int vid, int currhop, int dst, int prevhop) : vid_(vid), currhop_(currhop), prevhop_(prevhop), destination_(dst)
	{
	}

	bool getActionCommandResult(void) const
	{
		return action_result_;
	}

	int getCurrenthop(void) const
	{
		return currhop_;
	}

	int getNexthop(void) const
	{
		return nexthop_;
	}

	int getState(void) const
	{
		return vstate_;
	}

	void setState(State vstate)
	{
		vstate_ = vstate;
	}

	void connectEdgecomputer(int hop)
	{
		std::stringstream sstream;

		sstream.str("");
		sstream << hop;
		string str("new_connection_");
		str = str + sstream.str();
		ROS_INFO("str: %s", str.c_str());
		connection_client_ = nh_.serviceClient<sdn::NewConn>(str.c_str());
	}

	bool requestNexthop(char** argv)
	{
		sdn::NewConn srv;
		srv.request.vid = atoi(argv[1]);
		srv.request.src = atoi(argv[2]);
		srv.request.dst = atoi(argv[3]);
		srv.request.prev_hop_ip = atoi(argv[4]);

		if (connection_client_.call(srv))
		{
			ROS_INFO("next hop: %d", srv.response.next_hop_ip);
			nexthop_ = srv.response.next_hop_ip;
		}
		else
		{
			ROS_ERROR("Failed to call service new_connection");
			return false;
		}
		return true;
	}

	bool sendStateinfo(void)
	{
		string str = "state_info_";
		str = str + to_string(currhop_);
		vstate_client_ = nh_.serviceClient<sdn::StateInfo>(str.c_str());

		sdn::StateInfo srv;
		srv.request.vid = vid_;
		srv.request.vstate = getState();

		if (vstate_client_.call(srv))
		{
			ROS_INFO("stateinfo server status: [OK]");
		}
		else
		{
			ROS_ERROR("stateinfo server status: [FAILED]");
			return false;
		}
		return true;
	}

	void executeCallback(const sdn::VehicleCommandGoalConstPtr& goal)
	{
		ros::Rate r(1);
		sdn::VehicleCommandFeedback feedback;
		ROS_INFO("%s : Executing, %s", action_name_.c_str(), goal->command.c_str());

		int count = 0;
		while (count++ < 10)
		{
			feedback.vehicle_gps = "GPS data " + to_string(count);
			feedback.vehicle_controlinfo = "Control data " + to_string(count);
			ROS_INFO("publish feedback [%d]", count);
			server_->publishFeedback(feedback);
			r.sleep();
		}
		sdn::VehicleCommandResult result;
		result.command_completed = true;
		server_->setSucceeded(result);
	}

	void advertiseCommandAction(int hop)
	{
		string commandname = "command_";
		action_name_ = commandname + to_string(hop) + "_to_" + to_string(vid_);
		ROS_INFO("Vehicle Command Action Name: [%s]", action_name_.c_str());
		server_ = make_unique<Server>(nh_,
									  action_name_,
									  boost::bind(&Vehicle::executeCallback, this, _1),
									  false);
		server_->start();
	}

private:
	typedef actionlib::SimpleActionServer<sdn::VehicleCommandAction> Server;
	std::string action_name_;
	std::unique_ptr<Server> server_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle1");
	if (argc != 5)
	{
		ROS_INFO("usage: vehicle_id src dst prev_hop");
		return 1;
	}

	int start = atoi(argv[2]);
	int dst = atoi(argv[3]);
	int prevhop = atoi(argv[4]);
	Vehicle vehicle(1, start, dst, prevhop);

	while (ros::ok())
	{
		switch (vehicle.getState())
		{
		case Vehicle::VS_NEW:
		{
			ROS_INFO("Vehicle enter new connection");
			vehicle.connectEdgecomputer(vehicle.getCurrenthop());
			vehicle.requestNexthop(argv);
			vehicle.setState(Vehicle::VS_READYTO_COMMAND);
			break;
		}
		case Vehicle::VS_READYTO_COMMAND:
		{
			ROS_INFO("Vehicle ready to perform command");
			vehicle.sendStateinfo();
			vehicle.setState(Vehicle::VS_COMMAND);
			break;
		}
		case Vehicle::VS_COMMAND:
		{
			ROS_INFO("Vehicle is performing command");
			vehicle.advertiseCommandAction(vehicle.getCurrenthop());
			vehicle.setState(Vehicle::VS_REQ_NEXT_HOP);
			ros::spin();
			break;
		}
		case Vehicle::VS_REQ_NEXT_HOP:
		{
			ROS_INFO("Vehicle is going to connect to next hop");
		}
		default:
			return 0;
		}
	}
}
