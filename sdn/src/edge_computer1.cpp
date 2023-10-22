#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
// #include <vector>

#include "ros/ros.h"
#include "sdn/GpsInfo.h"
#include "sdn/NewConn.h"

using namespace std;

void gps_topic_callback(const sdn::GpsInfo::ConstPtr &msg);
bool connect(sdn::NewConn::Request &req, sdn::NewConn::Response &res);

class EdgeComputer
{
private:
	EdgeComputer() {}

private:
	ros::NodeHandle nh_;
	set<int> vehicles_;
	vector<ros::Subscriber> vehicles_gps_subscribers;
	unordered_map<int, int> lutable_;
	ros::ServiceServer conn_service_;

public:
	static EdgeComputer *instance_;
	enum EdgeComputerState
	{
		NEW_CONN,
		G_ADVERTISE_GPS_MSG,
		G_REGISTER_COMMAND_SRV,
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
		EdgeComputer::instance_->add_lutable(4, 2);  // TODO: 동적으로 채우기
		ROS_INFO("[EdgeComputer 1] Ready to connect");
		// controller에게 줄 topic을 advertise
	}

	void add_vehicle(int vid)
	{
		vehicles_.insert(vid);
	}

	void delete_vehicle(int vid)
	{
		vehicles_.erase(vid);
	}

	void subscribe_gps_topic(int vid)
	{
		stringstream sstream;
		sstream.str("");
		sstream << vid;
		string str = "gps_info_";
		str = str + sstream.str();
		ros::Subscriber sub = nh_.subscribe(str.c_str(), 1000, gps_topic_callback);
		vehicles_gps_subscribers.push_back(sub);
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

	void add_lutable(int dst, int nexthop)
	{
		// from controller
		lutable_[dst] = nexthop;
	}

	void delete_lutable(int dst)
	{
		lutable_.erase(dst);
	}

	void update_lutable(int dst, int nexthop)
	{
		// from controller
		lutable_[dst] = nexthop;
	}
};

void gps_topic_callback(const sdn::GpsInfo::ConstPtr &msg)
{
	// ROS_INFO("(%lf, %lf, %lf)", msg->latitude, msg->longitude, msg->altitude);
	ROS_INFO("GPS (%s)", msg->gpsinfo.c_str());
}

bool connect(sdn::NewConn::Request &req, sdn::NewConn::Response &res)
{
	int vid = req.vid;
	int src = req.src;
	int dst = req.dst;
	// register vid and subscribe vid's gps topic
	res.next_hop_ip = EdgeComputer::instance().lookup(dst);
	if (res.next_hop_ip < 0)
	{
		// get_next_hop_from(dst);
		// state change and request to controller
	}
	EdgeComputer::instance().subscribe_gps_topic(vid);
	ROS_INFO("Get destination: %d", req.dst);
	ROS_INFO("Get Next Hop Ip address: %d", res.next_hop_ip);
	return true;
}

EdgeComputer *EdgeComputer::instance_ = NULL;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edge_computer1");

	EdgeComputer &ec = EdgeComputer::instance();

	ec.initialize();

	ros::spin();
	return 0;
}
