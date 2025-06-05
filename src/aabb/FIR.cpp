#include "aabb/FIR.h"

using namespace std;
using namespace Eigen;

FIR_filter::FIR_filter(vector<float> b)
{
    this->b = b;
    this->num_taps = static_cast<uint16_t>(b.size());
}

FIR_filter::~FIR_filter()
{

}

vector<VectorXf> FIR_filter::filter(vector<VectorXf> DataIn)
{
    if (this->buffer.empty())
    {
        while (this->buffer.size() < this->b.size())
        {
            this->buffer.push_back(DataIn);
        }
    }

    vector<VectorXf> DataOut{8, VectorXf{4}};
    for (auto& Vertex : DataOut)
    {
        Vertex.fill(0.0);
    }
    this->buffer.push_front(DataIn);
    if (this->buffer.size() >  static_cast<uint16_t>(this->num_taps - 1) / 2)
    {
        this->buffer.pop_back();
        // convolve
        for (uint16_t i = 0; i < this->num_taps; i++)
        {
            for (uint8_t j = 0; j < 8; j++)
            {
                DataOut[j] += this->b[i] * this->buffer[i][j];
            }
        }
        // use current location
        // VectorXf Centre_current{ 4 };
        // VectorXf Centre_filtered{ 4 };
        // VectorXf shift_vector{ 4 };

        // Centre_current = 0.5 * (DataIn[1] + DataIn[7]);
        // Centre_filtered = 0.5 * (DataOut[1] + DataOut[7]);

        // shift_vector = Centre_current - Centre_filtered;

        // for (uint8_t i = 0; i < 8; i++)
        // {
        //     DataOut[i] += shift_vector;
        //     DataOut[i](3) = 1;
        // }

        return DataOut;
    }
    else
    {
        return DataIn;
    }
}
