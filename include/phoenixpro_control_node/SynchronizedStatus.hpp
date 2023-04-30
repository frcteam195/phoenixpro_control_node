#pragma once

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"

#include <mutex>

using namespace ctre::phoenixpro;

class SynchronizedStatus
{
public:
    std::vector<BaseStatusSignalValue*> get_status_signal_vector()
    {
        std::scoped_lock<std::recursive_mutex> lock(mutex);
        return status_signal_vector;
    }

protected:
    void add_status_signal_to_wait_vector(BaseStatusSignalValue* s)
    {
        std::scoped_lock<std::recursive_mutex> lock(mutex);
        status_signal_vector.push_back(s);
    }
private:
    std::recursive_mutex mutex;
    std::vector<BaseStatusSignalValue*> status_signal_vector;
};