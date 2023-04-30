#pragma once
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <map>
#include <tuple>
#include <boost/preprocessor.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)                              \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define X_DEFINE_MAP_FROM_ENUM(r, data, elem)                                                           \
    ( data::elem, rclcpp::Parameter() )

#define X_DEFINE_REVERSE_LOOKUP_MAP_FROM_ENUM(r, data, elem)                                            \
    ( BOOST_PP_STRINGIZE(elem), data::elem )

#define X_DEFINE_VECTOR_FROM_ENUM(r, data, elem)                                                        \
    ( p2s(data::elem) )

#define X_DEFINE_VECTOR_FROM_TYPES(r, data, type)                                                       \
    ( type )

#define X_PARAMETER_DECLATIONS(r, data, elem)                                                           \
    ( n->declare_parameter(p2s(data::elem), type) )

#define DEFINE_PARAMS(name, enumerators, types)                                                         \
    enum name {                                                                                         \
        BOOST_PP_SEQ_ENUM(enumerators)                                                                  \
    };                                                                                                  \
    std::map<std::string, name> __ParamNameReverseLookupMap =                                           \
        boost::assign::list_of< std::pair< std::string, name > >                                        \
        BOOST_PP_SEQ_FOR_EACH(X_DEFINE_REVERSE_LOOKUP_MAP_FROM_ENUM, name, enumerators);                \
                                                                                                        \
    typedef std::pair<name, rclcpp::Parameter> pair_type;                                               \
    typedef std::map< name, rclcpp::Parameter > map_type;                                               \
    map_type __ParamMap = boost::assign::list_of< pair_type >                                           \
        BOOST_PP_SEQ_FOR_EACH(X_DEFINE_MAP_FROM_ENUM, name, enumerators);                               \
                                                                                                        \
    inline std::string p2s(name v)                                                                      \
    {     /*param enum value to string*/                                                                \
        switch (v)                                                                                      \
        {                                                                                               \
            BOOST_PP_SEQ_FOR_EACH(                                                                      \
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,                                    \
                name,                                                                                   \
                enumerators                                                                             \
            )                                                                                           \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";                                   \
        }                                                                                               \
    }                                                                                                   \
                                                                                                        \
                                                                                                        \
    std::vector<std::string> __ParamNames = boost::assign::list_of< std::string >                       \
        BOOST_PP_SEQ_FOR_EACH(X_DEFINE_VECTOR_FROM_ENUM, name, enumerators);                            \
    std::vector<rclcpp::ParameterType> __ParamTypes = boost::assign::list_of< rclcpp::ParameterType >   \
        BOOST_PP_SEQ_FOR_EACH(X_DEFINE_VECTOR_FROM_TYPES, _, types);                                    \
                                                                                                        \
    inline std::map<name, rclcpp::Parameter>& load_parameters(rclcpp::Node* n) {                        \
        auto type_it = __ParamTypes.begin();                                                            \
        for (auto it = __ParamNames.begin(); it < __ParamNames.end(); ) {                               \
            n->declare_parameter((*it), (*type_it));                                                    \
            it++; type_it++;                                                                            \
        }                                                                                               \
        std::vector<rclcpp::Parameter> params = n->get_parameters(__ParamNames);                        \
        for (const rclcpp::Parameter& p : params) {                                                     \
            __ParamMap[__ParamNameReverseLookupMap[p.get_name()]] = p;                                  \
        }                                                                                               \
                                                                                                        \
        return __ParamMap;                                                                              \
    }                                                                                                   \
                                                                                                        \
    class ParameterizedNode : public rclcpp::Node                                                       \
    {                                                                                                   \
    public:                                                                                             \
        ParameterizedNode(std::string node_name) : rclcpp::Node(node_name)                              \
        {                                                                                               \
            params = load_parameters(this);                                                             \
        }                                                                                               \
        std::map<name, rclcpp::Parameter> params;                                                       \
    };                                                                                                  \
    
    