#pragma once
#include <string>
#include <iostream>


class Config
{
public:
    std::string name;
    /*
    std::string product = "Unknown";
    int cmd_addr = NULL;
    std::string cmd_proto = "v1";
    int sdk_addr = NULL;
    int video_stream_addr = NULL;
    int video_stream_port = NULL;
    std::string video_stream_proto = "tcp";
    int audio_stream_addr = NULL;
    int audio_stream_port = NULL;
    */

    Config();
    Config(std::string name);
    ~Config();

    friend std::ostream& operator<<(std::ostream& os, const Config& obj);
};