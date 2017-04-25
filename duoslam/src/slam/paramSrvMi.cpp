/*
 * paramSrvMi.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: liu
 */


#include "paramSrvMi.h"

ParamSrvMi* ParamSrvMi::_instanceMi = NULL;

void ParamSrvMi::configMi()
{

	addOption("submap_size",         			static_cast<int> (50),                      "the size of the submap");
	addOption("submap_overlap_size",         	static_cast<int> (10),                      "the size of the overlap part between submaps");
	addOption("submap_min_matches",         	static_cast<int> (100),                   	"minimum matched features between submaps");

	addOption("submap_downsample",     			static_cast<bool> (true),                   "wether to downsample the pcd, if yes, the octomap_resolution will be used");
	addOption("submap_dump",         			static_cast<bool> (true),                   "wether to save submaps");
	addOption("submap_saved_path",         		std::string("./submaps"),                   "the place to save submaps");

	addOption("globalmap_online_ros",         	static_cast<bool> (false),                  "whether to use ros to build global map online");
	addOption("submap_cloud_out_topic",    		std::string("/submap/cloud"),     			"topic to publish the submap cloud");
	addOption("submap_feature_des_out_topic",    		std::string("/submap/feature_des"),     			"topic to publish the submap cloud");
	addOption("submap_feature_loc_out_topic",    		std::string("/submap/feature_loc"),     			"topic to publish the submap cloud");

	addOption("globalmap_online_socket",        static_cast<bool> (false),                  "whether to use socket to build global map online");
	addOption("socket_host_ip",    				std::string("127.0.0.1"), 			    			"host ip");
	addOption("socket_host_port",    			static_cast<int>(8787), 			    			"host port");

	addOption("socket_client_ip",    			std::string("127.0.0.1"), 			    			"client ip");
	addOption("socket_client_port",    			static_cast<int>(6767), 			    			"client port");

    addOption("submap_number",                    static_cast<int>(10),                               "total submaps num");
    addOption("submap_step",                   static_cast<int>(1),                                "submap steps");
    addOption("submap_first",                    static_cast<int>(0),                               "the first submap id");
}

ParamSrvMi::ParamSrvMi(){

	  pre = ros::this_node::getName();
	  pre += "/config/";

	  configMi();
	  getValues();

}
ParamSrvMi::~ParamSrvMi(){
}


ParamSrvMi* ParamSrvMi::instanceMi()
{
	if(_instanceMi == NULL)
	{
		_instanceMi = new ParamSrvMi();
	}

	return _instanceMi;
}



void ParamSrvMi::addOption(std::string name, boost::any value, std::string description){
    config[name] = value;
    descriptions[name] = description;
}

/* Used by GUI */
std::string ParamSrvMi::getDescription(std::string param_name) {
  return descriptions[param_name];
}

void ParamSrvMi::getValues() {
  std::map<std::string, boost::any>::const_iterator itr;
  for (itr = config.begin(); itr != config.end(); ++itr) {
    std::string name = itr->first;
    if (itr->second.type() == typeid(std::string)) {
      config[name] = getFromParameterServer<std::string> (pre + name,
          boost::any_cast<std::string>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<std::string>(itr->second));
    } else if (itr->second.type() == typeid(int)) {
      config[name] = getFromParameterServer<int> (pre + name,
          boost::any_cast<int>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
    } else if (itr->second.type() == typeid(double)) {
      config[name] = getFromParameterServer<double> (pre + name,
          boost::any_cast<double>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
    } else if (itr->second.type() == typeid(bool)) {
      config[name] = getFromParameterServer<bool> (pre + name,
          boost::any_cast<bool>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
    }
  }
}
