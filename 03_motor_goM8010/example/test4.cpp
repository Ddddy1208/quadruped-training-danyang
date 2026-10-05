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
    const double KD_OUT = 3.0;

    const double KP = KP_OUT / (N * N);
    const double KD = KD_OUT / (N * N);

    const double DT = 0.002;
    const double MAX_SPEED = 10.0 * PI / 180.0;

    // 第3步：新零点物理正向移动30°
    const double BASE_OFFSET = -30.0 * PI / 180.0;

    // 相邻候选零点在输出端的间距
    const double ZERO_INTERVAL = 2.0 * PI / N;

    // 第2步旧零点，在第3步新坐标系中应该是 -30°
    const double STARTUP_EXPECTED = -30.0 * PI / 180.0;

    cmd.motorType = TYPE;
    data.motorType = TYPE;
    cmd.mode = queryMotorMode(TYPE, MotorMode::FOC);
    cmd.id = 0;

    // 先只读取位置
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;

    for (int i = 0; i < 20; i++) {
        serial.sendRecv(&cmd, &data);
        usleep(2000);
    }

    // 当前候选角度
    double q_candidate = data.q / N + BASE_OFFSET;

    // 自动判断落到了哪个相邻零点
    double zero_correction =
        std::round(
            (STARTUP_EXPECTED - q_candidate) / ZERO_INTERVAL
        ) * ZERO_INTERVAL;

    double q_now =
        data.q / N
        + BASE_OFFSET
        + zero_correction;

    // 启动时目标直接设成当前位置，避免突然运动，轨迹指令，电机下发的指令位置
    double q_ref = q_now;  
    double target = q_now;

    std::cout << "raw candidate = "
              << q_candidate * 180.0 / PI
              << " deg" << std::endl;

    std::cout << "zero correction = "
              << zero_correction * 180.0 / PI
              << " deg" << std::endl;

    std::cout << "corrected angle = "
              << q_now * 180.0 / PI
              << " deg" << std::endl;

    std::cout << "Input target angle (deg): " << std::endl;

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

        // 新坐标 -> 转子坐标
        cmd.q =
            (q_ref - BASE_OFFSET - zero_correction) * N;

        cmd.dq = 0.0;
        cmd.tau = 0.0;

        serial.sendRecv(&cmd, &data);

        // 转子反馈 -> 修正后的输出端坐标
        q_now =
            data.q / N
            + BASE_OFFSET
            + zero_correction;

        //检查键盘有没有输入
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