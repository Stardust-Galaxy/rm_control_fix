//
// Created by stardust on 2024/10/12.
//

#ifndef BUILD_XMLRPC_UTILITIES_HPP
#define BUILD_XMLRPC_UTILITIES_HPP

#include <rclcpp/rclcpp.hpp>
#include <xmlrpcpp/XmlRpcException.h>

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        const int tmp = value;
        return (double)tmp;
    }
    else
        return value;
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value, int field)
{
    ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    XmlRpc::XmlRpcValue value_xml = value[field];
    return xmlRpcGetDouble(value[field]);
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value, const std::string& field, double default_value)
{
    if (value.hasMember(field))
    {
        ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
                   (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
        return xmlRpcGetDouble(value[field]);
    }
    else
        return default_value;
}

#endif //BUILD_XMLRPC_UTILITIES_HPP
