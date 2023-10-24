
#include <string>
#include <unordered_map>

#include "ros/ros.h"
#include "sdn/GetFlowTable.h"
#include "sdn/ReqFlowTable.h"
#include "sdn/Traffic.h"

using namespace std;

void trafficTopicCallback(const sdn::Traffic::ConstPtr &msg);
bool setFlowTable(sdn::ReqFlowTable::Request &req,
				  sdn::ReqFlowTable::Response &res);

class Controller
{
private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_ = 1;
	ros::ServiceServer flow_table_service_;
	ros::ServiceClient get_flow_table_api_client_;
	unordered_map<int, ros::Subscriber> traffic_subscriber_map_;
	static Controller *instance_;
	unordered_map<int, int> traffic_map_;
	int map_[100][100];

	Controller()
	{
	}

public:
	static Controller &instance()
	{
		if (Controller::instance_ == NULL)
		{
			Controller::instance_ = new Controller();
		}
		return *instance_;
	}

	void initialize(void)
	{
		flow_table_service_ = nh_.advertiseService("flow_table_service", setFlowTable);
		ROS_INFO("Ready to connect");

		for (int i = 1; i <= 9; i++)
		{
			string topic_name = string("traffic_") + to_string(i);
			traffic_subscriber_map_[i] = nh_.subscribe<sdn::Traffic>(topic_name, 1000, trafficTopicCallback);
		}
	}

	int (*getMap(void))[100][100]
	{
		return &map_;
	}

	bool setLocalFlowTable(int ecid)
	{
		get_flow_table_api_client_ = nh_.serviceClient<sdn::GetFlowTable>("get_flow_table");
		sdn::GetFlowTable srv;

		srv.request.cid = 1;
		srv.request.ecid = ecid;

		if (get_flow_table_api_client_.call(srv))
		{
			ROS_INFO("Get local flow table from Application");
			for (int i = 0; i < 12; i++)
				map_[ecid][i] = srv.response.table[i];
		}
		else
		{
			ROS_ERROR("Failed to call get_flow_table");
			return false;
		}
		return true;
	}

	void setTrafficMap(int ecid, int vehicle_count)
	{
		traffic_map_[ecid] = vehicle_count;
	}
};
void trafficTopicCallback(const sdn::Traffic::ConstPtr &msg)
{
	const ros::Time timestamp = msg->timestamp;
	int ecid = msg->ecid;
	int vehicle_count = msg->vehicle_count;
	Controller::instance().setTrafficMap(ecid, vehicle_count);
}

bool setFlowTable(sdn::ReqFlowTable::Request &req,
				  sdn::ReqFlowTable::Response &res)
{
	ROS_INFO("Get EdgeComputer id: [%d]", req.ecid);

	Controller::instance().setLocalFlowTable(req.ecid);
	res.size = 9;
	int(*m)[100][100] = Controller::instance().getMap();

	for (int i = 1; i <= 9; i++)
		res.flow_table[i] = (*m)[req.ecid][i];
	return true;
}

Controller *Controller::instance_ = NULL;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	Controller &controller = Controller::instance();

	controller.initialize();

	ros::spin();

	return 0;
}