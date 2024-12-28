#include "periodics/ultrasonicsensor.hpp"
#define _18_chars 256

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor ultrasonicsensor
    *
    */
    CUltrasonicsensor::CUltrasonicsensor(
        std::chrono::milliseconds f_period,
        PinName f_pinTrg,
        PinName  f_pinEcho,
        UnbufferedSerial& f_serial)
    : utils::CTask(f_period)
    , m_pinTrg(f_pinTrg)
    , m_pinEcho(f_pinEcho)
    , m_serial(f_serial)
    , m_isActive(false)
    {
        /* constructor behaviour */
        m_pinTrg = 0;
    }

    /** @brief  CUltrasonicsensor class destructor
     */
    CUltrasonicsensor::~CUltrasonicsensor()
    {
    };

    void CUltrasonicsensor::serialCallbackULTRASONICcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
                m_isActive=(l_isActivate>=1);
                //bool_globalsV_battery_isActive = (l_isActivate>=1);
                sprintf(b,"1");
        }else{
            sprintf(b,"syntax error");
        }
    }

    uint16_t CUltrasonicsensor::MeasureDistance()
    {
        /* Measure distance behaviour */
        uint32_t time;
        uint16_t distance;

        Timer timer;
        timer.start();

        m_pinTrg = 0;
        wait_us(2);
        m_pinTrg = 1;
        wait_us(10);
        m_pinTrg = 0;

        // wait for the echo pin to go high
        Timer timeout;
        timeout.start();
        while (m_pinEcho == 0) {
            if (timeout.elapsed_time().count() > 30000) { // Timeout 30 ms
                return -1;
            }
        }

        // measure how long the echo pin was high
        timer.reset();
        while (m_pinEcho == 1) {
            if (timeout.elapsed_time().count() > 30000) { // Timeout 30 ms
                return -1;
            }
        }
        time = timer.elapsed_time().count(); // time in microseconds
        timer.stop();

        distance = time * 0.034f / 2.0f;

        return distance;

    }

    /* Run method */
    void CUltrasonicsensor::_run()
    {
        /* Run method behaviour */
        //if(!m_isActive) return;

        uint16_t distance = MeasureDistance();
        uint16_t distance_2 = distance + 11;
        uint16_t distance_3 = distance * 3;

        char buffer[_18_chars];

        snprintf(buffer, sizeof(buffer), "@ultrasonic:%d:%d:%d;;\r\n", distance, distance_2, distance_3);
        m_serial.write(buffer,strlen(buffer));
    }

}; // namespace periodics