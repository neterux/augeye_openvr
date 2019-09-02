#include "pupil.h"
#include <iostream>

Pupil::Pupil()
    : _ctx(1)
    , _req(_ctx, ZMQ_REQ)
    , _sub(_ctx, ZMQ_SUB)
{
    _req.connect("tcp://172.16.10.200:50020");
    const std::string m_sub = "SUB_PORT";
    _req.send(zmq::buffer(m_sub));
    zmq::message_t sub_port;
    _req.recv(&sub_port);
    
    _sub.connect("tcp://172.16.10.200:" + _decode(&sub_port));
    _sub.setsockopt(ZMQ_SUBSCRIBE, "pupil.", 6);
}

void Pupil::Get(float *center_x, float *center_y, int *nEye)
{
    zmq::message_t topic, msg;
    _sub.recv(&topic);
    _sub.recv(&msg);

    msgpack::object_handle oh = msgpack::unpack(static_cast<char*>(msg.data()), msg.size());
    msgpack::object obj = oh.get();
    //std::cout << obj << std::endl;

    pupil_data data;
    obj.convert(data);
    
    //std::cout << "confidence: " << data.confidence << std::endl;
    //std::cout << "center: (" << data.center[0] << ", " << data.center[1] << ")" << std::endl;
    if (data.confidence > 0.6)
    {
        if (data.id == 0)  // Right
        {
            *nEye = 1;  // vr::Eye_Right
            *center_x = 320 - data.center[0];
            *center_y = 240 - data.center[1];
        }
        if (data.id == 1)  // Left
        {
            *nEye = 0;  // vr::Eye_Left
            *center_x = data.center[0];
            *center_y = data.center[1];
        }
    }
}

void Pupil::Get(float* center_x, float* center_y, int nEye)
{
    zmq::message_t topic, msg;
    _sub.recv(&topic);
    _sub.recv(&msg);

    msgpack::object_handle oh = msgpack::unpack(static_cast<char*>(msg.data()), msg.size());
    msgpack::object obj = oh.get();
    //std::cout << obj << std::endl;

    pupil_data data;
    obj.convert(data);

    //std::cout << "confidence: " << data.confidence << std::endl;
    //std::cout << "center: (" << data.center[0] << ", " << data.center[1] << ")" << std::endl;
    if (data.id == nEye && data.confidence > 0.6)
    {
        if (nEye == 0)  // Right
        {
            *center_x = data.center[0];
            *center_y = data.center[1];
        }
        if (nEye == 1)  // Left
        {
            *center_x = 320 - data.center[0];
            *center_y = 240 - data.center[1];
        }
    }
}


std::string Pupil::_decode(zmq::message_t *msg)
{
    return std::string(static_cast<char*>(msg->data()), msg->size());
}