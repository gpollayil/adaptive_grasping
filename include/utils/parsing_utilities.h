#ifndef PARSING_UTILS_H
#define PARSING_UTILS_H

#include <XmlRpcValue.h>

/**
* @brief This h file contains utilities for parsing single parameters from 
* the ROS parameter server
*
*/

bool parseBoolParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name);

bool parseDoubleParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name);

bool parseIntParameter(XmlRpc::XmlRpcValue& params, int& param, std::string param_name);

bool parseStringParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name);

bool parseVectorParameter(XmlRpc::XmlRpcValue& params, std::vector<double>& param, std::string param_name, int min_size = 0);

bool parseMapParameter(XmlRpc::XmlRpcValue& params, std::map<int, std::string>& param, std::string param_name, std::vector< std::string > names_list);

#endif //PARSING_UTILS_H
