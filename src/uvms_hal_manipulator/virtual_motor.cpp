#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main()
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr*)&addr, sizeof(addr));

    std::cout << "✅ 虚拟电机已启动（ID=1），等待查询指令..." << std::endl;

    while (true)
    {
        struct can_frame frame;
        int n = read(s, &frame, sizeof(frame));

        if (n < 0) continue;

        // 这里修复：can_dlc 而不是 dlc
        if (frame.can_id == 1 && frame.can_dlc == 5 && frame.data[0] == 0x08)
        {
            std::cout << "📥 收到位置查询，回复虚拟位置 10.0 度" << std::endl;

            double target_deg = 10.0;
            const double ratio = 101.0;
            uint32_t raw = (target_deg * 65536.0 * ratio) / 360.0;

            struct can_frame reply;
            reply.can_id = 1;
            reply.can_dlc = 5;  // 修复：can_dlc
            reply.data[0] = 0x08;
            reply.data[1] = (raw >> 0) & 0xFF;
            reply.data[2] = (raw >> 8) & 0xFF;
            reply.data[3] = (raw >> 16) & 0xFF;
            reply.data[4] = (raw >> 24) & 0xFF;

            write(s, &reply, sizeof(reply));
        }
    }

    close(s);
    return 0;
}