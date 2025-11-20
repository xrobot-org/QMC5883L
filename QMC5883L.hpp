#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: QST QMC5883L 三轴磁力计驱动模块 / Driver module for QMC5883L 3-axis magnetometer
constructor_args:
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - topic_name: "qmc5883l_mag"
  - task_stack_depth: 1536
template_args: []
required_hardware: i2c_qmc5883l qmc5883l_int ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "message.hpp"
#include "transform.hpp"
#include <cstdint>

// ----- QMC5883L Registers (datasheet Table 13) -----
#define QMC5883L_REG_X_LSB 0x00
#define QMC5883L_REG_X_MSB 0x01
#define QMC5883L_REG_Y_LSB 0x02
#define QMC5883L_REG_Y_MSB 0x03
#define QMC5883L_REG_Z_LSB 0x04
#define QMC5883L_REG_Z_MSB 0x05
#define QMC5883L_REG_STATUS 0x06 // bits: [2]DOR [1]OVL [0]DRDY
#define QMC5883L_REG_TEMP_LSB 0x07
#define QMC5883L_REG_TEMP_MSB 0x08
#define QMC5883L_REG_CTRL1 0x09 // OSR[7:6] RNG[5:4] ODR[3:2] MODE[1:0]
#define QMC5883L_REG_CTRL2 0x0A // [7]SOFT_RST [6]ROL_PNT [0]INT_ENB(0=enable)
#define QMC5883L_REG_SET_RESET 0x0B // recommend 0x01
#define QMC5883L_REG_CHIP_ID 0x0D   // returns 0xFF

// ----- I2C address (7-bit) -----
#define QMC5883L_I2C_ADDR (0x0D << 1) // match your I2C API (8-bit address)

// ----- Driver configuration (match datasheet §7.1 Example) -----
#define QMC5883L_CTRL1_CONT_8G_200HZ_OSR512                                    \
  0x1D // OSR=512,RNG=±8G,ODR=200Hz,MODE=Continuous
#define QMC5883L_CTRL2_ROLLOVER_ENABLE                                         \
  0x40 // bit6=1, pointer roll-over; bit0=0 keeps INT enabled

// ±8G: 3000 LSB/G → 1000/3000 mG/LSB = 0.333333f
#define QMC5883L_SCALE_MG_PER_LSB (1000.0f / 3000.0f)

#define QMC5883L_MAG_RX_LEN 6

class QMC5883L : public LibXR::Application {
public:
  QMC5883L(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
           LibXR::Quaternion<float> &&rotation, const char *topic_name,
           size_t task_stack_depth)
      : rotation_(std::move(rotation)),
        topic_mag_(topic_name, sizeof(mag_data_)),
        int_drdy_(hw.template FindOrExit<LibXR::GPIO>({"qmc5883l_int"})),
        i2c_(hw.template FindOrExit<LibXR::I2C>({"i2c_qmc5883l"})),
        op_i2c_read_(sem_i2c_), op_i2c_write_(sem_i2c_),
        cmd_file_(LibXR::RamFS::CreateFile("qmc5883l", CommandFunc, this)) {

    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    int_drdy_->DisableInterrupt();
    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, QMC5883L *sensor) {
          sensor->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_drdy_->RegisterCallback(int_cb);

    while (!Init()) {
      XR_LOG_ERROR("QMC5883L: Init failed, retrying...\r\n");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("QMC5883L: Init succeeded.\r\n");
    thread_.Create(this, ThreadFunc, "qmc5883l_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  bool Init() {
    LibXR::Thread::Sleep(1);

    // CHIP ID should read 0xFF
    if (ReadReg(QMC5883L_REG_CHIP_ID) != 0xFF) {
      XR_LOG_ERROR("QMC5883L: wrong CHIP_ID.\r\n");
      return false;
    }

    // Enable pointer roll-over; keep INT enabled (INT_ENB=0)
    WriteReg(QMC5883L_REG_CTRL2, QMC5883L_CTRL2_ROLLOVER_ENABLE);

    // Set/Reset period recommended value
    WriteReg(QMC5883L_REG_SET_RESET, 0x01);

    // Continuous mode, ±8G, 200Hz, OSR=512
    WriteReg(QMC5883L_REG_CTRL1, QMC5883L_CTRL1_CONT_8G_200HZ_OSR512);

    LibXR::Thread::Sleep(5);

    // DRDY: default低，数据就绪拉高直到读数据（上升沿有效）
    int_drdy_->EnableInterrupt();
    return true;
  }

  static void ThreadFunc(QMC5883L *sensor) {
    while (true) {
      if (sensor->new_data_.Wait(100) == ErrorCode::OK) {
        sensor->ReadMagnetometer();
        sensor->ParseMagData();
        sensor->topic_mag_.Publish(sensor->mag_data_);
      } else {
        // 兜底轮询一次状态位，避免错过中断
        if (sensor->ReadReg(QMC5883L_REG_STATUS) & 0x01) {
          sensor->ReadMagnetometer();
          sensor->ParseMagData();
          sensor->topic_mag_.Publish(sensor->mag_data_);
        } else {
          XR_LOG_WARN("QMC5883L: DRDY timeout.\r\n");
        }
      }
    }
  }

  void ReadMagnetometer() {
    i2c_->MemRead(QMC5883L_I2C_ADDR, QMC5883L_REG_X_LSB, read_buffer_,
                  op_i2c_read_, LibXR::I2C::MemAddrLength::BYTE_8);
  }

  void ParseMagData() {
    std::array<int16_t, 3> raw;
    raw[0] = static_cast<int16_t>((read_buffer_[1] << 8) | read_buffer_[0]);
    raw[1] = static_cast<int16_t>((read_buffer_[3] << 8) | read_buffer_[2]);
    raw[2] = static_cast<int16_t>((read_buffer_[5] << 8) | read_buffer_[4]);

    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
      return;
    }

    // 标度：mG；如需 μT 可乘以 0.1
    Eigen::Matrix<float, 3, 1> vec;
    vec[0] = raw[0] * QMC5883L_SCALE_MG_PER_LSB;
    vec[1] = raw[1] * QMC5883L_SCALE_MG_PER_LSB;
    vec[2] = raw[2] * QMC5883L_SCALE_MG_PER_LSB;

    mag_data_ = rotation_ * vec;
  }

  uint8_t ReadReg(uint8_t reg) {
    uint8_t data = 0;
    i2c_->MemRead(QMC5883L_I2C_ADDR, reg, data, op_i2c_read_);
    return data;
  }

  void WriteReg(uint8_t reg, uint8_t val) {
    i2c_->MemWrite(QMC5883L_I2C_ADDR, reg, val, op_i2c_write_);
  }

  void OnMonitor(void) override {
    // 溢出位检测（可选）
    uint8_t st = ReadReg(QMC5883L_REG_STATUS);
    if (st & 0x02) { // OVL
      XR_LOG_WARN("QMC5883L: data overflow.\r\n");
    }
    if (std::isnan(mag_data_.x()) || std::isnan(mag_data_.y()) ||
        std::isnan(mag_data_.z()) || std::isinf(mag_data_.x()) ||
        std::isinf(mag_data_.y()) || std::isinf(mag_data_.z())) {
      XR_LOG_WARN("QMC5883L: NaN or Inf detected.\r\n");
    }
  }

  static int CommandFunc(QMC5883L *sensor, int argc, char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  show [time_ms] [interval_ms] - Print sensor data "
                           "periodically.\r\n");
    } else if (argc == 4 && strcmp(argv[1], "show") == 0) {
      int time_ms = atoi(argv[2]);
      int interval_ms = atoi(argv[3]);
      for (int i = 0; i < time_ms / interval_ms; i++) {
        LibXR::Thread::Sleep(interval_ms);
        LibXR::STDIO::Printf("Mag(mG): x=%f, y=%f, z=%f\r\n",
                             sensor->mag_data_.x(), sensor->mag_data_.y(),
                             sensor->mag_data_.z());
      }
    }
    return 0;
  }

private:
  LibXR::Quaternion<float> rotation_;
  Eigen::Matrix<float, 3, 1> mag_data_{0, 0, 0};
  LibXR::Topic topic_mag_;

  LibXR::GPIO *int_drdy_;
  LibXR::I2C *i2c_;

  LibXR::Semaphore sem_i2c_, new_data_;
  LibXR::ReadOperation op_i2c_read_;
  LibXR::WriteOperation op_i2c_write_;

  uint8_t read_buffer_[QMC5883L_MAG_RX_LEN] = {};

  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
