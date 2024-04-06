#ifndef _DIFF_CALCULATOR_H_
#define _DIFF_CALCULATOR_H_

#include "SRML.h"
#include "SecondButterworthLPF.h"

namespace DIFF_CAL_INTERNAL
{
    template <class Diff_Filter_Type>
    class DiffCalculator_Base : public myPIDTimer
    {
    public:
        /* 用模板构造函数，解决不同filter的构造函数参数不同的问题, 如果构造函数的参数不符合filter类型的构造函数传参，则会编译失败 */
        template <typename... Args>
        DiffCalculator_Base(Args... args) : filter(args...) {}

        inline float get_diff() const { return diff; }

        float calc(float input)
        {
            if (UpdataTimeStamp()) // 计算dt
                return 0;

            return calc(input, this->dt);
        }

        float calc(float input, float _dt)
        {
            float origin_diff;

            origin_diff = (input - last_data) / _dt; // 差分原始数据
            last_data = input;                       // 更新上次数据

            diff = get_filter_result(origin_diff);
            return diff;
        }

    protected:
        Diff_Filter_Type filter;
        inline virtual float get_filter_result(float data)
        {
            return filter.f(data);
        }
        float last_data = 0;
        float diff;
    };
}

template <class Diff_Filter_Type>
class DiffCalculator : public DIFF_CAL_INTERNAL::DiffCalculator_Base<Diff_Filter_Type>
{
public:
    /* 用模板构造函数，解决不同filter的构造函数参数不同的问题, 如果构造函数的参数不符合filter类型的构造函数传参，则会编译失败 */
    template <typename... Args>
    DiffCalculator(Args... args) : DIFF_CAL_INTERNAL::DiffCalculator_Base<Diff_Filter_Type>(args...) {}
};
#endif
