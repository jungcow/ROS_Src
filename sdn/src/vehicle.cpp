#include <cstdlib>
#include <sstream>

#include "ros/ros.h"
#include "sdn/GpsInfo.h"
#include "sdn/NewConn.h"

using namespace std;

class Vehicle
{
public:
	enum State
	{
		VS_NEW,
		G_ADVERTISE_GPS_MSG,
		G_REGISTER_COMMAND_SRV,
		VS_COMMAND,
		VS_REQ_NEXT_HOP,
		VS_DEL_PREV_HOP
	};

private:
	ros::NodeHandle nh_;
	ros::Publisher gps_pub_;
	ros::ServiceClient command_client_;
	ros::ServiceClient connection_client_;
	ros::Rate loop_rate_ = 1;
	int currhop_;
	int destination_;
	int vid_;
	int nexthop_ = 1;
	State vstate_ = VS_NEW;

public:
	Vehicle(int vid, int currhop, int dst) : vid_(vid), currhop_(currhop), destination_(dst)
	{
	}

	int get_currenthop(void) const
	{
		return currhop_;
	}

	int get_nexthop(void) const
	{
		return nexthop_;
	}

	int get_state(void) const
	{
		return vstate_;
	}

	void set_state(State vstate)
	{
		vstate_ = vstate;
	}

	void connect_edgecomputer(int hop)
	{
		std::stringstream sstream;

		sstream.str("");
		sstream << hop;
		string str("new_connection_");
		str = str + sstream.str();
		ROS_INFO("str: %s", str.c_str());
		connection_client_ = nh_.serviceClient<sdn::NewConn>(str.c_str());
	}

	bool request_nexthop(char **argv)
	{
		sdn::NewConn srv;
		srv.request.vid = atoi(argv[1]);
		srv.request.src = atoi(argv[2]);
		srv.request.dst = atoi(argv[3]);

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

	void advertise_gpsinfo(int qsize)
	{
		std::stringstream sstream;

		sstream.str("");
		sstream << vid_;
		string str("gps_info_");
		str = str + sstream.str();
		gps_pub_ = nh_.advertise<sdn::GpsInfo>(str.c_str(), qsize);
	}

	// void set_command_srv_client(int hop)
	// {
	// 	stringstream strstream;
	// 	strstream.str("command_");
	// 	strstream << currhop_;
	// 	command_client_ = nh_.serviceClient<sdn::Command>(strstream.str().c_str());
	// }

	void publish_gpsinfo(void)
	{
		sdn::GpsInfo gps_info;
		gps_info.gpsinfo = "[GPS Data]";
		// gps_info.latitude = 10.;
		// gps_info.longitude = 10.;
		// gps_info.altitude = 10.;

		// ROS_INFO("latitude: %lf, longitude: %lf, altitude: %lf",
		// 		 gps_info.latitude, gps_info.longitude, gps_info.altitude);
		ROS_INFO("publish data : %s", gps_info.gpsinfo.c_str());

		gps_pub_.publish(gps_info);
		ros::spinOnce();
		loop_rate_.sleep();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_sdn1");
	if (argc != 4)
	{
		ROS_INFO("usage: vehicle_id src dst");
		return 1;
	}

	int start = atoi(argv[2]);
	int dst = atoi(argv[3]);
	Vehicle vehicle(1, start, dst);

	while (ros::ok())
	{
		switch (vehicle.get_state())
		{
		case Vehicle::VS_NEW:
		{
			// TODO: edge computer에 따라 new_connection[i]라는 identifier를 추가해야함
			vehicle.connect_edgecomputer(vehicle.get_currenthop());
			vehicle.request_nexthop(argv);
			vehicle.set_state(Vehicle::G_ADVERTISE_GPS_MSG);
		}
		break;
		case Vehicle::G_ADVERTISE_GPS_MSG:
		{
			// TODO: vehicle에 따라 gps_info[i]라는 identifier를 추가해야함
			vehicle.advertise_gpsinfo(10);
			vehicle.publish_gpsinfo();
			vehicle.set_state(Vehicle::G_REGISTER_COMMAND_SRV);
			break;
		}
		case Vehicle::G_REGISTER_COMMAND_SRV:
		{
			// TODO: vehicle에 따라 command[i]라는 identifier를 추가해야함
			vehicle.publish_gpsinfo();

			// vehicle.set_command_srv_client(vehicle.get_currenthop());
			vehicle.set_state(Vehicle::VS_COMMAND);
			break;
		}
		case Vehicle::VS_COMMAND:
		{
			return 0;
			// vehicle.set_state(Vehicle::VS_REQ_NEXT_HOP);
			// break;
		}
		default:
			return 0;
		}
	}
}
