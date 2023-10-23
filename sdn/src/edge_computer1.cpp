#include <actionlib/client/simple_action_client.h>

#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros/ros.h"
#include "sdn/GpsInfo.h"
#include "sdn/NewConn.h"
#include "sdn/StateInfo.h"
#include "sdn/VehicleCommandAction.h"

using namespace std;

bool vstateCallback(sdn::StateInfo::Request &req, sdn::StateInfo::Response &res);
bool connect(sdn::NewConn::Request &req, sdn::NewConn::Response &res);

class EdgeComputer
{
private:
	EdgeComputer() {}

private:
	struct VehicleInfo
	{
		int src;
		int dst;
		int prevhop;
		int nexthop;
	};

	ros::NodeHandle nh_;
	unordered_map<int, VehicleInfo> vehicles_infos_;
	vector<ros::Subscriber> vehicles_state_subscribers;
	queue<int> vehicle_command_queue_;
	queue<int> vehicle_command_done_queue_;
	unordered_map<int, int> lutable_;
	ros::ServiceServer conn_service_;
	ros::ServiceServer vstate_service_;
	static EdgeComputer *instance_;

public:
	ros::Rate loop_rate = 10;

	enum EdgeComputerState
	{
		VS_NEW,
		VS_READYTO_COMMAND,
		VS_COMMAND,
		VS_REQ_NEXT_HOP,
		VS_DEL_PREV_HOP
	};

	static EdgeComputer &instance()
	{
		// 게으른 초기화
		if (EdgeComputer::instance_ == NULL)
		{
			EdgeComputer::instance_ = new EdgeComputer();
		}

		return *instance_;
	}

	void initialize(void)
	{
		conn_service_ = EdgeComputer::instance_->nh_.advertiseService("new_connection_1", connect);
		vstate_service_ = EdgeComputer::instance_->nh_.advertiseService("state_info_1", vstateCallback);
		// EdgeComputer::instance_->initialize_command_table();
		EdgeComputer::instance_->addLutable(4, 2);  // TODO: 동적으로 채우기
		ROS_INFO("[EdgeComputer 1] Ready to connect");
		// controller에게 줄 topic을 advertise
	}

	void saveVehicleInfo(int vid, int src, int dst, int prevhop, int nexthop)
	{
		VehicleInfo vinfo;
		vinfo.src = src;
		vinfo.dst = dst;
		vinfo.prevhop = prevhop;
		vinfo.nexthop = nexthop;
		vehicles_infos_[vid] = vinfo;
	}

	void deleteVehicleInfo(int vid)
	{
		vehicles_infos_.erase(vid);
	}

	int lookup(int dst)
	{
		auto it = lutable_.find(dst);
		if (it == lutable_.end())
		{
			return -1;
		}
		else
		{
			return it->second;
		}
	}

	void addLutable(int dst, int nexthop)
	{
		// from controller
		lutable_[dst] = nexthop;
	}

	void deleteLutable(int dst)
	{
		lutable_.erase(dst);
	}

	void updateLutable(int dst, int nexthop)
	{
		// from controller
		lutable_[dst] = nexthop;
	}

	string getCommand(int prevhop, int nexthop)
	{
		// return Command[prevhop][nexthop];
		return string("Left");
	}

	void pushVehicleCommandQueue(int vid)
	{
		vehicle_command_queue_.push(vid);
	}

	void pushVehicleCommandDoneQueue(int vid)
	{
		vehicle_command_done_queue_.push(vid);
	}

private:
	class ActionManager
	{
	private:
		typedef actionlib::SimpleActionClient<sdn::VehicleCommandAction> Client;
		std::unique_ptr<Client> client_;
		int vid;

	public:
		ActionManager(const std::string &action_name, int vid) : client_(new Client(action_name, true)), vid(vid)
		{
			client_->waitForServer();
		}

		void sendGoal(const std::string &command, EdgeComputer *edgecomputer)
		{
			sdn::VehicleCommandGoal goal;

			goal.command = command;

			ROS_INFO("Action goal is: [%s]", command.c_str());
			client_->sendGoal(goal,
							  boost::bind(&ActionManager::doneCallback, this, _1, _2),
							  Client::SimpleActiveCallback(),
							  boost::bind(&ActionManager::feedbackCallback, this, _1));
		}

		void feedbackCallback(const sdn::VehicleCommandFeedbackConstPtr &feedback)
		{
			ROS_INFO("Received feedback - GPS: [%s], ControlInfo: [%s]",
					 feedback->vehicle_gps.c_str(), feedback->vehicle_controlinfo.c_str());
		}

		void doneCallback(const actionlib::SimpleClientGoalState &state,
						  const sdn::VehicleCommandResultConstPtr &result)
		{
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Command completed: %s", result->command_completed ? "Yes" : "No");
				EdgeComputer::instance().pushVehicleCommandDoneQueue(vid);
			}
			else
			{
				ROS_ERROR("Action did not succeed.");
			}
		}
	};

private:
	unordered_map<int, std::unique_ptr<ActionManager> > action_manager_map_;

public:
	void deleteActionManager(int vid)
	{
		action_manager_map_.erase(vid);
	}

	void sendCommand(const string &command, const string &actionname, int vid)
	{
		action_manager_map_[vid] = std::make_unique<ActionManager>(actionname, vid);
		action_manager_map_[vid]->sendGoal(command, this);
	}

	void sendCommandAll(void)
	{
		ROS_INFO("command queue size: [%lu]", vehicle_command_queue_.size());
		while (!vehicle_command_done_queue_.empty())
		{
			int vid = vehicle_command_done_queue_.front();
			vehicle_command_done_queue_.pop();
			deleteActionManager(vid);
		}
		while (!vehicle_command_queue_.empty())
		{
			int vid = vehicle_command_queue_.front();
			vehicle_command_queue_.pop();
			VehicleInfo vinfo = vehicles_infos_[vid];
			string str = "command_1_to_" + to_string(vid);
			ROS_INFO("Action Command Name: [%s]", str.c_str());
			sendCommand(getCommand(vinfo.prevhop, vinfo.nexthop), str, vid);
		}
	}
};

bool vstateCallback(sdn::StateInfo::Request &req, sdn::StateInfo::Response &res)
{
	string str;
	int vid = req.vid;
	int vstate = req.vstate;
	switch (vstate)
	{
	case EdgeComputer::VS_READYTO_COMMAND:
	{
		str = "VS_READYTO_COMMAND";
		EdgeComputer::instance().pushVehicleCommandQueue(vid);
		break;
	}
	case EdgeComputer::VS_COMMAND:
	{
		str = "VS_COMMAND";
		break;
	}
	}
	res.status = true;
	ROS_INFO("Vehicle[%d]'s State : [%d]", vid, vstate);
	return true;
}

bool connect(sdn::NewConn::Request &req, sdn::NewConn::Response &res)
{
	int vid = req.vid;
	int src = req.src;
	int prevhop = req.prev_hop_ip;
	int dst = req.dst;
	res.next_hop_ip = EdgeComputer::instance().lookup(dst);
	if (res.next_hop_ip < 0)
	{
		// get_next_hop_from(dst);
		// state change and request to controller
	}
	// EdgeComputer::instance().subscribeStateTopic(vid);  // register vid and subscribe vid's state topic
	ROS_INFO("Get destination: %d", req.dst);
	ROS_INFO("Get Next Hop Ip address: %d", res.next_hop_ip);
	EdgeComputer::instance().saveVehicleInfo(vid, src, dst, prevhop, res.next_hop_ip);
	return true;
}

EdgeComputer *EdgeComputer::instance_ = NULL;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edge_computer1");

	EdgeComputer &ec = EdgeComputer::instance();
	ec.initialize();

	while (ros::ok())
	{
		ec.sendCommandAll();
		ros::spinOnce();
		ec.loop_rate.sleep();
	}
	return 0;
}
