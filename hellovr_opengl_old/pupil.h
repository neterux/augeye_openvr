#include <zmq.hpp>
#include <string>

class Pupil
{
public:
    Pupil();
    //~Pupil();

    std::string Get();

    std::string addr = "172.16.10.200";
    std::string req_port = "50020";

private:
    zmq::context_t _context;
    zmq::socket_t _request;
    zmq::socket_t _subscribe;

};