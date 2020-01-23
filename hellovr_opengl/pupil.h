#pragma once
#include <zmq.hpp>
#include <msgpack.hpp>
#include <string>

#define MSGPACK_USE_DEFINE_MAP

struct ellipse
{
    std::vector<float> center;
    std::vector<float> axes;
    float angle;

    MSGPACK_DEFINE_MAP(center, axes, angle);
};

class pupil_data : public ellipse
{
public:
    pupil_data() {};

    std::string topic;
    float confidence;
    float diameter;
    std::vector<float> norm_pos;
    float timestamp;
    std::string method;
    int id;

    MSGPACK_DEFINE_MAP(topic, confidence, MSGPACK_BASE_MAP(ellipse),
        diameter, norm_pos, timestamp, method, id);
};
/*
2D
{
    'topic': 'pupil.0',
    'confidence': 0.8991729021072388,
    'ellipse': {
        'center': [126.77586364746094, 83.90132904052734],
        'axes': [46.58835983276367, 53.25790786743164],
        'angle': 132.2717437744141
    },
    'diameter': 53.25790786743164,
    'norm_pos': [0.39617457389831545, 0.6504111289978027],
    'timestamp': 26030.59621,
    'method': '2d c++',
    'id': 0
}

3D
{
    'topic': 'pupil.1',
    'circle_3d': {
        'center': [1.0122264874630353, 3.89931849979625, 61.30663836507062],
        'normal': [-0.07920782043531578, -0.5430637911395056, -0.8359472710255559],
        'radius': 2.72626396567654
    },
    'confidence': 0.8002374324808417,
    'timestamp': 82036.779041,
    'diameter_3d': 5.45252793135308,
    'ellipse': {
        'center': [170.32407607548984, 160.01461944782847],
        'axes': [48.09961794890503, 55.160152331723914],
        'angle': 82.52120633672234
    },
    'norm_pos': [0.5322627377359057, 0.33327241896738136],
    'diameter': 55.160152331723914,
    'sphere': {
        'center': [1.9627203326868248, 10.416083993470318, 71.33800561737729],
        'radius': 12.0
    },
    'projected_sphere': {
        'center': [177.05804074188205, 210.52638940579652],
        'axes': [208.5844687025476, 208.5844687025476],
        'angle': 90.0
    },
    'model_confidence': 0.2596772054974994,
    'model_id': 35,
    'model_birth_timestamp': 81984.388241,
    'theta': 0.9967147935943158,
    'phi': -1.6652664566816933,
    'method': '3d c++',
    'id': 1
}
*/

class Pupil
{
public:
    Pupil();
    void Get(float* center_x, float* center_y, int nEye);

private:
    zmq::context_t _ctx;
    zmq::socket_t _req;
    zmq::socket_t _sub;

    std::string _decode(zmq::message_t *msg);
};