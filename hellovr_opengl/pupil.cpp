#include "pupil.h"
#include <iostream>

Pupil::Pupil()
    : _ctx(1)
    , _req(_ctx, ZMQ_REQ)
    , _sub(_ctx, ZMQ_SUB)
{
    _req.connect("tcp://172.16.200.29:50020");
    const std::string m_sub = "SUB_PORT";
    _req.send(zmq::buffer(m_sub));
    zmq::message_t sub_port;
    _req.recv(&sub_port);
    
    _sub.connect("tcp://172.16.200.29:" + _decode(&sub_port));
    _sub.setsockopt(ZMQ_SUBSCRIBE, "pupil.", 6);
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

    // data.id = 0, nEye = 1: Right eye
    // data.id = 1, nEye = 0: Left eye
    if (data.id == !nEye && data.confidence > 0.4)
    {
        *center_x = data.center[0];
        *center_y = data.center[1];
    }
}


std::string Pupil::_decode(zmq::message_t *msg)
{
    return std::string(static_cast<char*>(msg->data()), msg->size());
}