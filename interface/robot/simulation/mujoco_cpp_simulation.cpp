/**
 * @file pybullet_interface.hpp
 * @brief communicate with pybullet
 * @author mazunwang
 * @version 1.0
 * @date 2024-09-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <array>
#include <arpa/inet.h>
#include <unistd.h>

constexpr char XML_PATH[] = "../../../third_party/URDF_model/lite3_mjcf/mjcf/Lite3.xml";
constexpr int DOF = 12;
constexpr int UDP_PORT = 20001;
constexpr int CTRL_PORT = 30010;
constexpr float DT = 0.001f;

mjModel* m = nullptr;
mjData* d = nullptr;
GLFWwindow* window = nullptr;

std::array<float, DOF> kp{}, kd{}, qd{}, dqd{}, tau_ff{};
std::array<float, DOF> last_tau{};

// Quaternion to Euler conversion
std::array<float, 3> quat_to_euler(const mjtNum* q) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float pitch = asinf (2.0f * (w * y - z * x));
    float yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return {roll, pitch, yaw};
}

void udp_receiver() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));

    char buf[240];
    while (true) {
        ssize_t len = recv(sockfd, buf, sizeof(buf), 0);
        if (len == 240) {
            std::memcpy(kp.data(),      buf + 0,     48);
            std::memcpy(qd.data(),      buf + 48,    48);
            std::memcpy(kd.data(),      buf + 96,    48);
            std::memcpy(dqd.data(),     buf + 144,   48);
            std::memcpy(tau_ff.data(),  buf + 192,   48);
        }
    }
}

void apply_control() {
    for (int i = 0; i < DOF; i++) {
        float q_err  = qd[i]  - static_cast<float>(d->qpos[7 + i]);
        float dq_err = dqd[i] - static_cast<float>(d->qvel[6 + i]);
        d->ctrl[i] = kp[i] * q_err + kd[i] * dq_err + tau_ff[i];
        last_tau[i] = d->ctrl[i];
    }
}

void send_state(int sockfd, sockaddr_in& target, double timestamp, const std::array<float, 3>& acc_world) {
    const mjtNum* quat = &d->qpos[3];
    auto rpy = quat_to_euler(quat);

    std::array<float, 3> linvel{d->qvel[3], d->qvel[4], d->qvel[5]};
    std::array<float, 3> angvel{d->qvel[0], d->qvel[1], d->qvel[2]};

    mjtNum rotmat[9];
    mjv_defaultCamera(nullptr);
    mju_quat2Mat(rotmat, quat);

    float R[3][3];
    for (int i = 0; i < 9; ++i)
        R[i / 3][i % 3] = static_cast<float>(rotmat[i]);

    std::array<float, 3> body_acc{}, body_omega{};
    for (int i = 0; i < 3; ++i) {
        body_acc[i] = R[0][i]*acc_world[0] + R[1][i]*acc_world[1] + R[2][i]*acc_world[2];
        body_omega[i] = R[0][i]*angvel[0] + R[1][i]*angvel[1] + R[2][i]*angvel[2];
    }

    std::array<float, DOF> q{}, dq{};
    for (int i = 0; i < DOF; i++) {
        q[i] = static_cast<float>(d->qpos[7 + i]);
        dq[i]= static_cast<float>(d->qvel[6 + i]);
    }

    std::array<float, 1 + 3 + 3 + 3 + DOF*3> payload;
    payload[0] = static_cast<float>(timestamp);
    std::memcpy(&payload[1],       rpy.data(),     12);
    std::memcpy(&payload[4],       body_acc.data(),12);
    std::memcpy(&payload[7],       body_omega.data(), 12);
    std::memcpy(&payload[10],      q.data(),       48);
    std::memcpy(&payload[22],      dq.data(),      48);
    std::memcpy(&payload[34],      last_tau.data(),48);

    sendto(sockfd, payload.data(), payload.size()*sizeof(float), 0,
           (struct sockaddr*)&target, sizeof(target));
}

int main() {
    char error[1000];
    m = mj_loadXML(XML_PATH, nullptr, error, 1000);
    if (!m) {
        std::cerr << "Could not load model: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // Set initial pose
    for (int i = 0; i < DOF; i++)
        d->qpos[7 + i] = (i % 3 == 0) ? 0.0 : ((i % 3 == 1) ? -1.35453 : 2.54948);

    mj_forward(m, d);

    std::thread(recv_thread).detach();
    recv_thread = std::thread(udp_receiver);

    // UDP send init
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(CTRL_PORT);
    inet_pton(AF_INET, "127.0.0.1", &target.sin_addr);

    // Viewer init
    if (!glfwInit()) return -1;
    window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
    if (!window) return -1;
    glfwMakeContextCurrent(window);

    mjvScene scn; mjv_defaultScene(&scn);
    mjrContext con; mjr_defaultContext(&con);
    mjvCamera cam; mjv_defaultCamera(&cam);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    mjv_makeScene(m, &scn, 1000);

    int step = 0;
    std::array<float, 3> last_linvel{};

    while (!glfwWindowShouldClose(window)) {
        apply_control();
        mj_step(m, d);

        std::array<float, 3> linvel{d->qvel[3], d->qvel[4], d->qvel[5]};
        std::array<float, 3> acc_world;
        for (int i = 0; i < 3; ++i)
            acc_world[i] = (linvel[i] - last_linvel[i]) / DT;
        last_linvel = linvel;

        send_state(sockfd, target, step * DT, acc_world);

        mjv_updateScene(m, d, &cam, nullptr, nullptr, mjCAT_ALL, &scn);
        mjr_render(mjvRect{0, 0, 1200, 900}, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();

        step++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
