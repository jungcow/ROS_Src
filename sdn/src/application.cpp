#include "ros/ros.h"
#include "sdn/GetFlowTable.h"

int map_[100][100];

bool getFlowTable(sdn::GetFlowTable::Request &req,
				  sdn::GetFlowTable::Response &res)
{
	ROS_INFO("path_planning_server listening");
	int cid = req.cid;
	int ecid = req.ecid;

	int p12[] = {2, 3, 6, 9};
	int p14[] = {4, 5, 7, 8};
	for (int i = 0; i < 4; i++)
	{
		map_[1][p12[i]] = 2;
		map_[1][p14[i]] = 4;
	}
	map_[2][1] = 1;
	int p23[] = {3, 6, 9};
	int p25[] = {4, 5, 7, 8};
	for (int i = 0; i < 3; i++)
	{
		map_[2][p23[i]] = 3;
		map_[2][p25[i]] = 5;
	}
	map_[2][p25[3]] = 5;

	int p32[] = {1, 2, 4, 7};
	int p36[] = {5, 6, 8, 9};
	for (int i = 0; i < 4; i++)
	{
		map_[3][p32[i]] = 2;
		map_[3][p36[i]] = 6;
	}

	int p41[] = {1, 2, 3};
	int p45[] = {5, 6};
	int p47[] = {7, 8, 9};
	for (int i = 0; i < 2; i++)
	{
		map_[4][p41[i]] = 1;
		map_[4][p45[i]] = 5;
		map_[4][p47[i]] = 7;
	}
	map_[4][p41[2]] = 1;
	map_[4][p47[2]] = 7;

	int p52[] = {2, 3};
	int p54[] = {1, 4};
	int p56[] = {6, 9};
	int p58[] = {7, 8};
	for (int i = 0; i < 2; i++)
	{
		map_[5][p52[i]] = 2;
		map_[5][p54[i]] = 4;
		map_[5][p56[i]] = 6;
		map_[5][p58[i]] = 8;
	}

	int p63[] = {1, 2, 3};
	int p65[] = {4, 5};
	int p67[] = {7, 8, 9};
	for (int i = 0; i < 2; i++)
	{
		map_[6][p63[i]] = 3;
		map_[6][p65[i]] = 5;
		map_[6][p67[i]] = 9;
	}
	map_[6][p63[2]] = 3;
	map_[6][p67[2]] = 9;

	int p74[] = {1, 2, 4, 5};
	int p78[] = {3, 6, 8, 9};
	for (int i = 0; i < 4; i++)
	{
		map_[7][p74[i]] = 4;
		map_[7][p78[i]] = 8;
	}

	int p85[] = {2, 5};
	int p87[] = {1, 4, 7};
	int p89[] = {3, 6, 9};
	for (int i = 0; i < 2; i++)
	{
		map_[8][p85[i]] = 5;
		map_[8][p87[i]] = 7;
		map_[8][p89[i]] = 9;
	}
	map_[8][p87[2]] = 7;
	map_[8][p89[2]] = 9;

	int p96[] = {2, 3, 5, 6};
	int p98[] = {1, 4, 7, 8};
	for (int i = 0; i < 4; i++)
	{
		map_[9][p96[i]] = 6;
		map_[9][p98[i]] = 8;
	}
	for (int i = 1; i <= 9; i++)
	{
		map_[i][i] = 0;
	}

	for (int i = 1; i <= 9; i++)
		res.table[i] = map_[ecid][i];

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "application");

	ros::NodeHandle nh_;
	ros::ServiceServer path_planning_server = nh_.advertiseService("get_flow_table", getFlowTable);

	ros::spin();
}