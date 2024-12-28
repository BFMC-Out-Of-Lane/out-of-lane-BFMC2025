#ifndef ULTRASONICSENSOR_HPP
#define ULTRASONICSENSOR_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
/* The mbed library */
#include <mbed.h>
#include <brain/globalsv.hpp>
#include <cstdint>
#include <cstdio>

namespace periodics
{
   /**
    * @brief Class ultrasonicsensor
    *
    */
    class CUltrasonicsensor: public utils::CTask
    {
        public:
            /* Constructor */
            CUltrasonicsensor(
                std::chrono::milliseconds f_period,
                PinName f_pinTrg,
                PinName  f_pinEcho,
                UnbufferedSerial& f_serial
            );
            /* Destructor */
            ~CUltrasonicsensor();
            /* Serial callback implementation */
            void serialCallbackULTRASONICcommand(char const * a, char * b);

            uint16_t MeasureDistance();
        private:
            /* private variables & method member */

            /* Run method */
            virtual void        _run();

            /* @brief Pin trigger and echo */
            mbed::DigitalOut m_pinTrg;
            mbed::DigitalIn m_pinEcho;

            /* @brief Serial communication obj.  */
            UnbufferedSerial&          m_serial;

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CUltrasonicsensor
}; // namespace periodics

#endif // ULTRASONICSENSOR_HPP
