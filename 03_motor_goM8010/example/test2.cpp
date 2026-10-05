#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <cmath>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

int main()
{
    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;

    const MotorType TYPE = MotorType::GO_M8010_6;
    const double N = queryGearRatio(TYPE);
    const double PI = 3.141592653589793;

    const double KP_OUT = 80.0;
    const double KD_OUT = 3;

    const double KP = KP_OUT / (N * N);
    const double KD = KD_OUT / (N * N);

    const double DT = 0.002;

    // 输出端最大速度：10°/s
    const double MAX_SPEED = 10.0 * PI / 180.0;

    cmd.motorType = TYPE;
    data.motorType = TYPE;
    cmd.mode = queryMotorMode(TYPE, MotorMode::FOC);
    cmd.id = 0;

    // 先读取当前位置
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;

    for (int i = 0; i < 20; i++) {
        serial.sendRecv(&cmd, &data);
        usleep(2000);
    }

    double q_ref = data.q / N;
    double target = 0.0;

    std::cout << "Current output angle = "
              << q_ref * 180.0 / PI
              << " deg" << std::endl;

    std::cout << "Returning to 0 deg..." << std::endl;

    while (true)
    {
        double error = target - q_ref;
        double step = MAX_SPEED * DT;

        if (error > step)
            q_ref += step;
        else if (error < -step)
            q_ref -= step;
        else
            q_ref = target;

        cmd.kp = KP;
        cmd.kd = KD;
        cmd.q = q_ref * N;
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        serial.sendRecv(&cmd, &data);

        double q_now = data.q / N;

        // 检查终端有没有输入
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;

        if (poll(&pfd, 1, 0) > 0)
        {
            double degree;

            if (std::cin >> degree)
            {
                if (degree >= -30.0 && degree <= 30.0)
                {
                    target = degree * PI / 180.0;

                    std::cout << "New target = "
                              << degree
                              << " deg" << std::endl;
                }
                else
                {
                    std::cout << "Please input -30 ~ 30 deg"
                              << std::endl;
                }
            }
        }

        static int count = 0;

        if (++count % 500 == 0)
        {
            std::cout
                << "q = "
                << q_now * 180.0 / PI
                << " deg, target = "
                << target * 180.0 / PI
                << " deg"
                << std::endl;
        }

        usleep(2000);
    }
}