#include "pupil.h"


Pupil::Pupil()
    : _request(_context, zmq::socket_type::req)
    , _subscribe(_context, zmq::socket_type::req)
{
    _request.connect("tcp://172.16.10.200:50020");
    const std::string m_sub = "SUB_PORT";
    _request.send(zmq::buffer(m_sub));
    zmq::message_t sub_port;
    _request.recv(sub_port);

    _subscribe.connect("tcp://172.16.10.200:" + sub_port.str());
    const std::string m_topic = "pupil.";
    _subscribe.setsockopt((int)zmq::socket_type::sub, zmq::buffer(m_topic));
}

std::string Pupil::Get()
{
    zmq::message_t msg;
    _subscribe.recv(msg);
    return msg.str();
}