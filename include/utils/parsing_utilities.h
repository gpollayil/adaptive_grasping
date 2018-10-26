#ifndef PARSING_UTILS_H
#define PARSING_UTILS_H

#include <XmlRpcValue.h>
#include "ros/ros.h"

/**
* @brief This h file contains utilities for parsing single parameters from 
* the ROS parameter server
*
*/

/* PARSEBOOLPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeBoolean);

    // Re-convert to bool and print out
    param = (bool) params[param_name];
    ROS_INFO_STREAM("Read the bool " << param_name << " = " << (param? "true." : "false."));

    return true;
};

/* PARSEINTPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, int& param, std::string param_name){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt);

    // Re-convert to bool and print out
    param = (int) params[param_name];
    ROS_INFO_STREAM("Read the int " << param_name << " = " << param << ".");

    return true;
};

/* PARSEDOUBLEPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Re-convert to bool and print out
    param = (double) params[param_name];
    ROS_INFO_STREAM("Read the double " << param_name << " = " << param << ".");

    return true;
};

/* PARSESTRINGPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeString);

    // Re-convert to bool and print out
    param = (std::string) params[param_name];
    ROS_INFO_STREAM("Read the string " << param_name << " = " << param << ".");

    return true;
}

/* PARSEVECTORPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::vector<double>& param, std::string param_name, int min_size = 0){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray);

    // Check if the vector has the minimum required length
    if(params[param_name].size() < min_size){
        ROS_ERROR_STREAM("The vector " << param_name <<" is shorter than required.");
        return false;
    }

    // Clearing the vector to return and filling up
    param.clear();
    for(int i=0; i< params[param_name].size(); i++){
        param.push_back((double) params[param_name][i]);
    }
    ROS_INFO_STREAM("Read the vector " << param_name << " of size " << params[param_name].size() << ".");

    return true;
};

/* PARSEINTSTRINGMAPPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::map<int, std::string>& param, std::string param_name){
    // Checking if there is a parameter with the requested name in the bunch of parsed parameters
    if(!params.hasMember(param_name)){
        ROS_ERROR_STREAM("No value found for " << param_name <<" parameter.");
        return false;
    }

    // Make sure that the parameter is of the correct type
    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeStruct);

    // Creating temporary map and filling it up
    std::map<int, std::string> tmp_param;
    for(auto it : param){
        tmp_param[it->first] = it->second;
    }

    // Check is the temporary map is empty
    
};

#endif //PARSING_UTILS_H