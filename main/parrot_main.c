/*  Flexible pipeline playback with different music

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"

#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "wav_encoder.h"
#include "wav_decoder.h"
#include "raw_stream.h"
#include "filter_resample.h"

#include "board.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_button.h"
#include "periph_adc_button.h"
#include "periph_touch.h"

#include <math.h>

#include "goertzel.h"

#define true 1
#define false 0
#define FILE_PATH "/sdcard/Parrot/parrot.wav"
#define MAX_RECORD_LENGTH 10 //seconds


static const char *TAG = "TALKING_PARROT";

static esp_periph_set_handle_t set;
static audio_event_iface_handle_t evt;
audio_pipeline_handle_t parrot_pipeline = NULL;
audio_element_handle_t i2s_stream_reader_el, i2s_stream_writer_el, filter_el, raw_read_el,
                            wav_encoder_el, wav_decoder_el, fatfs_stream_writer_el,
                            fatfs_stream_reader_el;

bool start_up = true;
bool record = false;

/*
   VOORBEELDCODE: Onderstaande callback komt uit het goertzel voorbeeld.
   BY ME: Het wisselen van start_up naar record while loop. 
*/
void goertzel_callback(struct goertzel_data_t *filter, float result)
{
    goertzel_data_t *filt = (goertzel_data_t *)filter;
    float logVal = 10.0f * log10f(result);

    // Detection filter. Only above 25 dB(A)
    if (logVal > 25.0f)
    {
        // ESP_LOGI(TAG, "[Goertzel] Callback Freq: %d Hz amplitude: %.2f", filt->target_frequency, 10.0f * log10f(result));
        record = true;
        start_up = false;
    }
}

/*
    BY ME: Deze methodes zijn voor het creëren van alle audio elementen
*/
// #region create audio elements
static audio_element_handle_t create_i2s_stream_reader()
{
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    return i2s_stream_init(&i2s_cfg);
}

static audio_element_handle_t create_i2s_stream_writer()
{
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    return i2s_stream_init(&i2s_cfg);
}

static audio_element_handle_t create_goertzel_filter()
{
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = AUDIO_SAMPLE_RATE;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = GOERTZEL_SAMPLE_RATE_HZ;
    rsp_cfg.dest_ch = 1;
    return rsp_filter_init(&rsp_cfg);
}

static audio_element_handle_t create_goertzel_raw_read()
{
    raw_stream_cfg_t raw_cfg = {
        .out_rb_size = 8 * 1024,
        .type = AUDIO_STREAM_READER,
    };
    return raw_stream_init(&raw_cfg);
}

static audio_element_handle_t create_fatfs_stream_reader()
{
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    return fatfs_stream_init(&fatfs_cfg);
}

static audio_element_handle_t create_fatfs_stream_writer()
{
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_WRITER;
    return fatfs_stream_init(&fatfs_cfg);
}

static audio_element_handle_t create_wav_encoder()
{
    wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    return wav_encoder_init(&wav_cfg);
}

static audio_element_handle_t create_wav_decoder()
{
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    return wav_decoder_init(&wav_cfg);
}
// #endregion


/*
    BY ME: Onderstaande 3 methodes zijn voor het versimpelen van de code.
    Er wordt per onderdeel van de applicatie een relink gemaakt voor het gebruik van de pipeline.
    Aan het begin van luisteren op een klap, het opnemen en het afspelen zal een van deze methode worden aangeroepen.
*/
// #region reset pipelines and relink
void reset_to_record()
{
    audio_pipeline_pause(parrot_pipeline);

    audio_pipeline_reset_elements(parrot_pipeline);
    audio_pipeline_relink(parrot_pipeline, (const char *[]){"i2s_stream_reader", "wav_encoder", "fatfs_stream_writer"}, 3);
    audio_pipeline_set_listener(parrot_pipeline, evt);

    audio_pipeline_run(parrot_pipeline);
    audio_pipeline_resume(parrot_pipeline);
}

void reset_to_play()
{
    audio_pipeline_pause(parrot_pipeline);

    audio_pipeline_reset_elements(parrot_pipeline);
    audio_pipeline_relink(parrot_pipeline, (const char *[]){"i2s_stream_writer", "wav_decoder", "fatfs_stream_reader"}, 3);
    audio_pipeline_set_listener(parrot_pipeline, evt);

    audio_pipeline_run(parrot_pipeline);
    audio_pipeline_resume(parrot_pipeline);
}

void reset_to_detect()
{
    audio_pipeline_pause(parrot_pipeline);

    audio_pipeline_reset_elements(parrot_pipeline);
    audio_pipeline_relink(parrot_pipeline, (const char *[]){"i2s_stream_reader", "filter", "raw_read"}, 3);
    audio_pipeline_set_listener(parrot_pipeline, evt);

    audio_pipeline_run(parrot_pipeline);
    audio_pipeline_resume(parrot_pipeline);
}
// #endregion


void run_parrot_pipeline()
{
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    parrot_pipeline = audio_pipeline_init(&pipeline_cfg);

    /* BY ME: Alle audio elementen creëren */

    ESP_LOGI(TAG, "[ 1 ] Create all audio elements for playback pipeline");
    i2s_stream_reader_el = create_i2s_stream_reader();
    i2s_stream_writer_el = create_i2s_stream_writer();
    filter_el = create_goertzel_filter();
    raw_read_el = create_goertzel_raw_read();
    wav_encoder_el = create_wav_encoder();
    wav_decoder_el = create_wav_decoder();
    fatfs_stream_writer_el = create_fatfs_stream_writer();
    fatfs_stream_reader_el = create_fatfs_stream_reader();

    /* BY ME: Alle audio elementen registreren op de pipeline */

    ESP_LOGI(TAG, "[ 2 ] Register all audio elements to playback pipeline");
    audio_pipeline_register(parrot_pipeline, i2s_stream_reader_el, "i2s_stream_reader");
    audio_pipeline_register(parrot_pipeline, i2s_stream_writer_el, "i2s_stream_writer");
    audio_pipeline_register(parrot_pipeline, filter_el, "filter");
    audio_pipeline_register(parrot_pipeline, raw_read_el, "raw_read");
    audio_pipeline_register(parrot_pipeline, wav_encoder_el, "wav_encoder");
    audio_pipeline_register(parrot_pipeline, wav_decoder_el, "wav_decoder");
    audio_pipeline_register(parrot_pipeline, fatfs_stream_writer_el, "fatfs_stream_writer");
    audio_pipeline_register(parrot_pipeline, fatfs_stream_reader_el, "fatfs_stream_reader");

    audio_element_set_uri(fatfs_stream_writer_el, FILE_PATH);
    audio_element_set_uri(fatfs_stream_reader_el, FILE_PATH);

    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    evt = audio_event_iface_init(&evt_cfg);

    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    /* VOORBEELDCODE: Gebruikt voor het detecteren van bepaalde frequenties */

    // Config goertzel filters
    goertzel_data_t **goertzel_filt = goertzel_malloc(GOERTZEL_N_DETECTION); // Alloc mem
    // Apply configuration for all filters
    for (int i = 0; i < GOERTZEL_N_DETECTION; i++)
    {
        goertzel_data_t *currFilter = goertzel_filt[i];
        currFilter->samples = GOERTZEL_BUFFER_LENGTH;
        currFilter->sample_rate = GOERTZEL_SAMPLE_RATE_HZ;
        currFilter->target_frequency = GOERTZEL_DETECT_FREQUENCIES[i];
        currFilter->goertzel_cb = &goertzel_callback;
    }

    // Initialize goertzel filters
    goertzel_init_configs(goertzel_filt, GOERTZEL_N_DETECTION);

    ESP_LOGI(TAG, "[ 4 ] Start playback pipeline");
    audio_pipeline_link(parrot_pipeline, (const char *[]){"i2s_stream_reader", "filter", "raw_read"}, 3);
    audio_pipeline_run(parrot_pipeline);

    /*  BY ME: Deze while is verantwoordelijk voor het detecteren van een klap geluid.
        Zodra het klapgeluid is gesignaleerd zal in de goertzel_callback start_up op false worden gezet.
        De onderstaande while loop stopt dan dus, maar wordt later weer opgestart.
        VOORBEELDCODE: De onderstaande code is uit het goertzel voorbeeld.
    */
    while (start_up == true)
    {
        reset_to_detect();

        bool noError = true;
        int16_t *raw_buff = (int16_t *)malloc(GOERTZEL_BUFFER_LENGTH * sizeof(short));
        if (raw_buff == NULL)
        {
            ESP_LOGE(TAG, "Memory allocation failed!");
            noError = false;
        }

        while (noError && start_up == true)
        {
            raw_stream_read(raw_read_el, (char *)raw_buff, GOERTZEL_BUFFER_LENGTH * sizeof(short));

            // process Goertzel Samples
            goertzel_proces(goertzel_filt, GOERTZEL_N_DETECTION, raw_buff, GOERTZEL_BUFFER_LENGTH);
        }

        if (raw_buff != NULL)
        {
            free(raw_buff);
            raw_buff = NULL;
        }
    }

    /*
        BY ME: Deze while loop is verantwoordelijk voor het opnemen en afspelen van de gebruikers stem.
        In de goertzel_callback wordt deze while loop op true gezet, er wordt dan gelijk geluid opgenomen.
        Er kan met MAX_RECORD_LENGTH ingesteld worden hoelang dit moet gebeuren.
        Het wordt tijdelijk weggeschreven naar een .wav file op de sd kaart.
        Hierna wordt er vanaf diezelfde file afgespeeld wat er is opgenomen.
        Na het afspelen wordt de start_up while weer op true gezet en deze op false.
    */ 
    int second_recorded = 0;
    while (record == true)
    {
        /*
            BY ME: Het onderdeel voor het opnemen van de gebruikers stem.
        */
            reset_to_record();
            ESP_LOGI(TAG, "Start recording for %i seconds", MAX_RECORD_LENGTH);

            audio_event_iface_msg_t msg;
            ESP_LOGI(TAG, "Recording...");

            if (audio_event_iface_listen(evt, &msg, 1000 / portTICK_RATE_MS) != ESP_OK)
            {
                second_recorded++;
                ESP_LOGI(TAG, "[ * ] Recording ... %d", second_recorded);
                if (second_recorded >= MAX_RECORD_LENGTH)
                {
                    break;
                }
                continue;
            }

            /* Stop when the last pipeline element (i2s_stream_reader in this case) receives stop event */
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == i2s_stream_reader_el && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED)))
            {
                ESP_LOGW(TAG, "[ * ] Stop event received");
                break;
            }
        


        /*
            BY ME: Het onderdeel voor het afspelen van de gebruikers stem.
        */
            reset_to_play();
            ESP_LOGI(TAG, "Playback the recording");

            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == wav_decoder_el
                && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) 
                {
                audio_element_info_t music_info = {0};
                audio_element_getinfo(wav_decoder_el, &music_info);

                ESP_LOGI(TAG, "[ * ] Receive music info from wav decoder, sample_rates=%d, bits=%d, ch=%d",
                        music_info.sample_rates, music_info.bits, music_info.channels);

                audio_element_setinfo(i2s_stream_writer_el, &music_info);
                i2s_stream_set_clk(i2s_stream_writer_el, music_info.sample_rates, music_info.bits, music_info.channels);
                continue;
            }
            /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == i2s_stream_writer_el
                && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
                && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) 
                {
                ESP_LOGW(TAG, "[ * ] Stop event received");
                break;
            }

        start_up = true;
        record = false;

    }

    /*
        Hier worden alle onderdelen afgehandeld zodat er geen memory leaks voorkomen.
        Alles wordt gedeinit en unregistered. 
    */
    ESP_LOGI(TAG, "[ 5 ] Destroy goertzel");
    goertzel_free(goertzel_filt);

    ESP_LOGI(TAG, "[ 6 ] Stop playback pipeline");
    audio_pipeline_terminate(parrot_pipeline);
    audio_pipeline_unregister_more(parrot_pipeline, i2s_stream_reader_el, i2s_stream_writer_el,
                                   wav_decoder_el, wav_encoder_el, filter_el, raw_read_el,
                                   fatfs_stream_reader_el, fatfs_stream_writer_el, NULL);

    audio_pipeline_remove_listener(parrot_pipeline);
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);
    audio_event_iface_destroy(evt);

    audio_element_deinit(i2s_stream_reader_el);
    audio_element_deinit(i2s_stream_writer_el);
    audio_element_deinit(wav_decoder_el);
    audio_element_deinit(wav_encoder_el);
    audio_element_deinit(filter_el);
    audio_element_deinit(raw_read_el);
    audio_element_deinit(fatfs_stream_reader_el);
    audio_element_deinit(fatfs_stream_writer_el);
}

void app_main(void)
{
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set);

    // Setup audio codec
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    run_parrot_pipeline();
    esp_periph_set_destroy(set);
}
