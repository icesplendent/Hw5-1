#include "DynamicMessageBufferFactory.h"
#include "UARTTransport.h"
#include "bbcar_control_server.h"
#include "drivers/DigitalOut.h"
#include "erpc_basic_codec.h"
#include "erpc_crc16.h"
#include "erpc_simple_server.h"
#include "mbed.h"
// Uncomment for actual BB Car operations
#include <cmath>

#include "bbcar.h"

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmOut pin9(D9), pin10(D10);
PwmIn pin11(D11), pin12(D12);
DigitalInOut pin8(D8);
BBCar car(pin9, pin11, pin10, pin12, servo_ticker, servo_feedback_ticker);
parallax_ping ping1(pin8);
float angle1 = 0;
float angle2 = 0;
float distanceBetween = 0;
float d1 = 0;
float d2 = 0;
const double PI = 3.1415926535897932;

/**
 * Macros for setting console flow control.
 */
#define CONSOLE_FLOWCONTROL_RTS 1
#define CONSOLE_FLOWCONTROL_CTS 2
#define CONSOLE_FLOWCONTROL_RTSCTS 3
#define mbed_console_concat_(x) CONSOLE_FLOWCONTROL_##x
#define mbed_console_concat(x) mbed_console_concat_(x)
#define CONSOLE_FLOWCONTROL \
    mbed_console_concat(MBED_CONF_TARGET_CONSOLE_UART_FLOW_CONTROL)

mbed::DigitalOut led1(LED1, 1);
mbed::DigitalOut led2(LED2, 1);
mbed::DigitalOut led3(LED3, 1);
mbed::DigitalOut* leds[] = {&led1, &led2, &led3};
// Uncomment for actual BB Car operations
BBCar* cars[] = {&car};  // Control only one car

/****** erpc declarations *******/

void stop(uint8_t car) {
    if (car == 1) {  // there is only one car
        *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        (*cars[car - 1]).stop();
        printf("Car %d stop.\n", car);
    }
}

void goStraight(uint8_t car, int32_t speed) {
    if (car == 1) {  // there is only one car
        *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        (*cars[car - 1]).goStraight(speed);
        printf("Car %d go straight at speed %d.\n", car, speed);
    }
}

void turn(uint8_t car, int32_t speed, double factor) {
    if (car == 1) {  // there is only one car
        *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        (*cars[car - 1]).turn(speed, factor);
        printf("Car %d turn at speed %d with a factor of %f.\n", car, speed,
               factor);
    }
}

void detect(uint8_t car, int32_t speed, double factor) {
    angle1 = 0;
    angle2 = 0;
    if (car == 1) {  // there is only one car
        *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        // turn left
        while (1) {
            if ((float)ping1 > 15) {
                led1 = 1;
                // keep turning
            } else {
                led1 = 0;
                // stop turning
                stop(1);
                break;
            }
            turn(1, 40, 1);
            angle1 -= 2.2;
            ThisThread::sleep_for(100ms);
        }
        printf("(turn left) angle is %f.\n", angle1);
        d1 = ping1;

        stop(1);
        ThisThread::sleep_for(100ms);

        // turn right
        while (1) {
            if ((float)ping1 > 15) {
                led1 = 1;
                // keep turning
            } else if (angle1 < 0) {
                led1 = 1;
                // keep turning
            } else {
                led1 = 0;
                // stop turning
                stop(1);
                break;
            }
            turn(1, 40, -1);
            angle2 += 2.2;
            angle1 += 2.2;
            ThisThread::sleep_for(100ms);
        }
        d2 = ping1;

        printf("%f\n", 2 * d1 * d2 * cos((angle2 + angle1) / 180 * PI));
        distanceBetween = sqrt(d1 * d1 + d2 * d2 -
                               2 * d1 * d2 * cos((angle2 + angle1) / 180 * PI));
        printf("d1 = %f, d2 = %f, angle is %f. distance is %f\n", d1, d2,
               angle2 + angle1, distanceBetween);
    }
}

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** LED service */
BBCarService_service car_control_service;

int main(void) {
    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

    // Set up hardware flow control, if needed
#if CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTS
    uart_transport.set_flow_control(mbed::SerialBase::RTS, STDIO_UART_RTS, NC);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_CTS
    uart_transport.set_flow_control(mbed::SerialBase::CTS, NC, STDIO_UART_CTS);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTSCTS
    uart_transport.set_flow_control(mbed::SerialBase::RTSCTS, STDIO_UART_RTS,
                                    STDIO_UART_CTS);
#endif

    printf("Initializing server.\n");
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);

    // Add the led service to the server
    printf("Adding BBCar server.\n");
    rpc_server.addService(&car_control_service);

    // Run the server. This should never exit
    printf("Running server.\n");
    rpc_server.run();
}
