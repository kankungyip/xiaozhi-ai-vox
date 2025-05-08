#pragma once
#include <driver/i2s_pdm.h>
#include <esp_log.h>

#include <cmath>
#include <cstring>

#include "audio_codecs/no_audio_codec.h"

class AecAudioCodec : public NoAudioCodec {
  private:
    // ref buffer used for aec
    std::vector<int16_t> ref_buffer_;
    int read_pos_ = 0;
    int write_pos_ = 0;

    int Write(const int16_t *data, int samples) {
        if (output_enabled_) {
            std::vector<int32_t> buffer(samples);

            // output_volume_: 0-100
            // volume_factor_: 0-65536
            int32_t volume_factor = pow(double(output_volume_) / 100.0, 2) * 65536;
            for (int i = 0; i < samples; i++) {
                int64_t temp = int64_t(data[i]) * volume_factor; // 使用 int64_t 进行乘法运算
                if (temp > INT32_MAX) {
                    buffer[i] = INT32_MAX;
                } else if (temp < INT32_MIN) {
                    buffer[i] = INT32_MIN;
                } else {
                    buffer[i] = static_cast<int32_t>(temp);
                }
            }

            size_t bytes_written;
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2s_channel_write(tx_handle_, buffer.data(), samples * sizeof(int32_t), &bytes_written, portMAX_DELAY));

            // 板子不支持硬件回采，采用缓存播放缓冲来实现回声消除
            if (input_reference_) {
                if (write_pos_ - read_pos_ + samples > ref_buffer_.size()) {
                    assert(ref_buffer_.size() >= samples);
                    // 写溢出，只保留最近的数据
                    read_pos_ = write_pos_ + samples - ref_buffer_.size();
                }
                if (read_pos_) {
                    if (write_pos_ != read_pos_) {
                        memmove(ref_buffer_.data(), ref_buffer_.data() + read_pos_,
                                (write_pos_ - read_pos_) * sizeof(int16_t));
                    }
                    write_pos_ -= read_pos_;
                    read_pos_ = 0;
                }
                memcpy(&ref_buffer_[write_pos_], data, samples * sizeof(int16_t));
                write_pos_ += samples;
            }
        }
        return samples;
    }

    int Read(int16_t *dest, int samples) {
        if (input_enabled_) {
            int size = samples / input_channels_;
            std::vector<int16_t> buffer(size);

            size_t bytes_read;
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2s_channel_read(rx_handle_, buffer.data(), size * sizeof(int16_t), &bytes_read, portMAX_DELAY));

            if (!input_reference_) {
                memcpy(dest, buffer.data(), size * sizeof(int16_t));
            } else {
                int i = 0;
                int j = 0;
                while (i < samples) {
                    // mic data
                    dest[i++] = buffer[j++];

                    // ref data
                    dest[i++] = read_pos_ < write_pos_ ? ref_buffer_[read_pos_++] : 0;
                }

                if (read_pos_ == write_pos_) {
                    read_pos_ = write_pos_ = 0;
                }
            }
        }
        return samples;
    }

  public:
    AecAudioCodec(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws,
                  gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_din, bool input_reference) {
        duplex_ = false;                    // 是否双工
        input_reference_ = input_reference; // 是否使用参考输入，实现回声消除
        if (input_reference_) {
            ref_buffer_.resize(960 * 2);
        }
        input_channels_ = 1 + input_reference_; // 输入通道数："M" / "MR"
        input_sample_rate_ = input_sample_rate;
        output_sample_rate_ = output_sample_rate;

        // Create a new channel for speaker
        i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)1, I2S_ROLE_MASTER);
        tx_chan_cfg.dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM;
        tx_chan_cfg.dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM;
        tx_chan_cfg.auto_clear_after_cb = true;
        tx_chan_cfg.auto_clear_before_cb = false;
        tx_chan_cfg.intr_priority = 0;
        ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle_, NULL));

        i2s_std_config_t tx_std_cfg = {
            .clk_cfg =
                {
                    .sample_rate_hz = (uint32_t)output_sample_rate_,
                    .clk_src = I2S_CLK_SRC_DEFAULT,
                    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
#ifdef I2S_HW_VERSION_2
                    .ext_clk_freq_hz = 0,
#endif

                },
            .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg =
                {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = spk_bclk,
                    .ws = spk_ws,
                    .dout = spk_dout,
                    .din = I2S_GPIO_UNUSED,
                    .invert_flags =
                        {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &tx_std_cfg));
#if SOC_I2S_SUPPORTS_PDM_RX
        // Create a new channel for MIC in PDM mode
        i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)0, I2S_ROLE_MASTER);
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle_));
        i2s_pdm_rx_config_t pdm_rx_cfg = {
            .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)input_sample_rate_),
            /* The data bit-width of PDM mode is fixed to 16 */
            .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg =
                {
                    .clk = mic_sck,
                    .din = mic_din,

                    .invert_flags =
                        {
                            .clk_inv = false,
                        },
                },
        };
        ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle_, &pdm_rx_cfg));
#else
        ESP_LOGE("AecAudioCodec", "PDM is not supported");
#endif
        ESP_LOGI("AecAudioCodec", "Simplex channels created");
    }

    void SetOutputVolume(int volume) {
        output_volume_ = volume;
        AudioCodec::SetOutputVolume(volume);
    }
};
