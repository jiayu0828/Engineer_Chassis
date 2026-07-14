#ifndef PYRO_DATABOARD_H
#define PYRO_DATABOARD_H

#include <stdint.h>
#include "cmsis_os.h"
#include "task.h"

namespace pyro{
    typedef enum{
        UNSIGNED_INT = 0,
        SIGNED_INT,
        FLOAT
    }
    data_type_t;

    typedef union{
        uint32_t data_ui;
        int32_t data_si;
        float data_f;
    }
    genenral_data_t;
    //通用类型数据包，但这么做会不会有点浪费？
    class topic
    {
        public:
            typedef enum{
                DATA_OK,
                DATA_ERROR,
                DATA_INVALID
            }
            data_status_t;
            topic(const char *name,data_type_t type);
            ~topic();

            bool operator==(const topic &other);//重载等于运算符，便于比较
            data_status_t write(genenral_data_t& data);
            data_status_t read(genenral_data_t* data);
            const char* get_name();
            TickType_t get_timestamp();//得到目前的时间，用来比较数据是否新鲜
        private:
            char *_name;
            genenral_data_t _data;
            TickType_t _timestamp;
            data_type_t _type;
            SemaphoreHandle_t _semaphore;
            bool _valid;
            //这些东西都封装起来，有点类似_ctx的感觉？
    };
    class databoard
    {
        const static uint8_t DATABOARD_CHANNEL_COUNT = 48;
        public:
            databoard();
            ~databoard();
            
            uint32_t create_topic(const char *name,data_type_t type);
            uint32_t get_topic_id(const char *name);
            bool delete_topic(const char *name);
            topic::data_status_t write_topic(uint32_t id,genenral_data_t data);
            topic::data_status_t read(uint32_t id,genenral_data_t* data,TickType_t& timestamp);
        private:
            topic *_topics[DATABOARD_CHANNEL_COUNT + 1];
            SemaphoreHandle_t _semaphore;
    };
    

}
#endif