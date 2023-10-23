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
#include "sdn/ReqFlowTable.h"
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
	// vector<ros::Subscriber> vehicles_state_subscribers;
	queue<int> vehicle_command_queue_;
	queue<int> vehicle_command_done_queue_;
	unordered_map<int, int> lutable_;
	ros::ServiceServer conn_service_;
	ros::ServiceServer vstate_service_;
	ros::ServiceClient flowtable_service_client_;
	static EdgeComputer *instance_;
	string name_;
	int ecid_;

public:
	ros::Rate loop_rate = 10;

	enum EdgeComputerState
	{
		VS_NEW,
		VS_READYTO_COMMAND,
		VS_COMMAND,
		VS_REQ_NEXT_HOP,
		VS_EXIT
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

	void initialize(int ecid)
	{
		ecid_ = ecid;
		name_ = string("[edge_computer[") + to_string(ecid_) + string("]");
		string name = "EdgeComputer " + to_string(ecid_);
		string conn_name = "new_connection_" + to_string(ecid_);
		string stateinfo_name = "state_info_" + to_string(ecid_);

		initializeCommandTable();

		conn_service_ = EdgeComputer::instance_->nh_.advertiseService(conn_name, connect);
		ROS_INFO("%s Ready to connect", name_.c_str());

		vstate_service_ = EdgeComputer::instance_->nh_.advertiseService(stateinfo_name, vstateCallback);
		ROS_INFO("%s Ready to get vehicle's states", name_.c_str());

		// controller에게 줄 topic을 advertise
	}

	bool initializeCommandTable(void)
	{
		flowtable_service_client_ = nh_.serviceClient<sdn::ReqFlowTable>("flow_table_service");

		sdn::ReqFlowTable srv;

		srv.request.ecid = ecid_;
		if (flowtable_service_client_.call(srv))
		{
			int tablesize = srv.response.size;
			for (int i = 1; i <= tablesize; i++)
			{
				lutable_[i] = srv.response.flow_table[i];
			}
		}
		else
		{
			ROS_ERROR("Failed to call flow_table_service");
			return false;
		}
		return true;
	}

	const string &getName(void)
	{
		return name_;
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
		lutable_[dst] = nexthop;
		// announce to controller
	}

	void deleteLutable(int dst)
	{
		lutable_.erase(dst);
		// announce to controller
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

			ROS_INFO("%s Action goal is: [%s]", EdgeComputer::instance().getName().c_str(), command.c_str());
			client_->sendGoal(goal,
							  boost::bind(&ActionManager::doneCallback, this, _1, _2),
							  Client::SimpleActiveCallback(),
							  boost::bind(&ActionManager::feedbackCallback, this, _1));
		}

		void feedbackCallback(const sdn::VehicleCommandFeedbackConstPtr &feedback)
		{
			ROS_INFO("%s Received feedback - GPS: [%s], ControlInfo: [%s]",
					 EdgeComputer::instance().getName().c_str(), feedback->vehicle_gps.c_str(), feedback->vehicle_controlinfo.c_str());
		}

		void doneCallback(const actionlib::SimpleClientGoalState &state,
						  const sdn::VehicleCommandResultConstPtr &result)
		{
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("%s Command completed: %s",
						 EdgeComputer::instance().getName().c_str(),
						 result->command_completed ? "Yes" : "No");
				EdgeComputer::instance().pushVehicleCommandDoneQueue(vid);
			}
			else
			{
				ROS_ERROR("%s Action did not succeed.", EdgeComputer::instance().getName().c_str());
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
			string str = "command_" + to_string(ecid_) + "_to_" + to_string(vid);
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
	ROS_INFO("%s Vehicle[%d]'s State : [%d]", EdgeComputer::instance().getName().c_str(), vid, vstate);
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
	ROS_INFO("%s Get destination: %d", EdgeComputer::instance().getName().c_str(), req.dst);
	ROS_INFO("%s Get Next Hop Ip address: %d", EdgeComputer::instance().getName().c_str(), res.next_hop_ip);
	EdgeComputer::instance().saveVehicleInfo(vid, src, dst, prevhop, res.next_hop_ip);
	return true;
}

EdgeComputer *EdgeComputer::instance_ = NULL;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edge_computer");

	if (argc != 2)
	{
		ROS_INFO("usage: edgecomputer_id");
		return 1;
	}
	int ecid = atoi(argv[1]);

	EdgeComputer &ec = EdgeComputer::instance();
	ec.initialize(ecid);

	while (ros::ok())
	{
		ec.sendCommandAll();
		ros::spinOnce();
		ec.loop_rate.sleep();
	}
	return 0;
}
