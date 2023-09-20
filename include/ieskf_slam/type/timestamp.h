#pragma once
#include <iostream>
namespace IESKFSlam
{
    class TimeStamp
    {
    private:
        uint64_t nsec_;
        double sec_;
    public:
        TimeStamp(uint64_t insec = 0){
            nsec_ = insec;
            sec_ = static_cast<double>(insec)/1e9;
        }
        void fromSec(double isec){
            sec_ = isec;
            nsec_ =static_cast<uint64_t>(isec*1e9);
        }
        void fromNsec(uint64_t insec = 0){
            nsec_ = insec;
            sec_ = static_cast<double>(insec)/1e9;
        }
        const uint64_t & nsec()const {return nsec_;}
        const double & sec()const {return sec_;}
        // . 小数型统一定义为秒
    };
    
} 