#pragma once 

#include <cstdint>

namespace dart::hardware::device {

class IIM42622Driver {
public:
    virtual ~IIM42622Driver() = default;

    inline bool init() {
        //initialization code to be filled
        return true;
    }

    inline int16_t read_raw() {
        // to be adjusted
        return 1024;
    }

    inline void reset() {
        
    }

};
} // namespace dart::hardware