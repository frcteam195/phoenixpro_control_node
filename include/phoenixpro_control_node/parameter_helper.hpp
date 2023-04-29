#pragma once
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <map>
#include <tuple>
#include <boost/preprocessor.hpp>
#include <boost/assign/list_of.hpp>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define X_DEFINE_MAP_FROM_ENUM(r, data, elem)                                 \
    ( data::elem, rclcpp::Parameter() )

#define DEFINE_PARAMS(name, enumerators)                                      \
    enum name {                                                               \
        BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
                                                                              \
    typedef std::pair<name, rclcpp::Parameter> pair_type;                     \
    typedef std::map< name, rclcpp::Parameter > map_type;                     \
    const map_type ParamMap = boost::assign::list_of< pair_type >             \
        BOOST_PP_SEQ_FOR_EACH(X_DEFINE_MAP_FROM_ENUM, name, enumerators);     \
                                                                              \
    inline const char* p2s(name v)                                            \
    {     /*param enum value to string*/                                      \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
        }                                                                     \
    }

