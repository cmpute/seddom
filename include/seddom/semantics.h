#pragma once

#include <string>
#include <tuple>
#include <cassert>

#define _scolor(i) (float(i)) / 255.f
#define _scolor3(r, g, b) std::make_tuple(_scolor(r), _scolor(g), _scolor(b))
#define _color_ignore _scolor3(0, 0, 0)

namespace seddom
{
    typedef std::tuple<float, float, float> RGB;

    // TODO: the unlabelled category is not the same as "free"

    struct ToyDataset
    {
        static constexpr size_t NumClass = 5;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0:
                return _color_ignore;
            case 1: 
                return _scolor3(255, 0, 0);
            case 2:
                return _scolor3(70, 130, 180);
            case 3:
                return _scolor3(218, 112, 214);
            case 4: // noise
                return _scolor3(128, 128, 128);
            }
        }
    };

    struct SemanticKITTI
    {
        static constexpr size_t NumClass = 20;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0: // unlabeled
                return _color_ignore;
            case 1: // car
                return _scolor3(245, 150, 100);
            case 2: // bicycle
                return _scolor3(245, 230, 100);
            case 3: // motorcycle
                return _scolor3(150, 60,  30);
            case 4: // truck
                return _scolor3(180, 30,  80);
            case 5: // other-vehicle
                return _scolor3(255, 80,  100);
            case 6: // person
                return _scolor3(30,  30,  255);
            case 7: // bicyclist
                return _scolor3(200, 40,  1);
            case 8: // motorcyclist
                return _scolor3(90,  30,  150);
            case 9: // road
                return _scolor3(255, 0,   255);
            case 10: // parking
                return _scolor3(255, 150, 255);
            case 11: // sidewalk
                return _scolor3(75,  0,   75);
            case 12: // other-ground
                return _scolor3(75,  0,   175);
            case 13: // building
                return _scolor3(0,   200, 255);
            case 14: // fence
                return _scolor3(50,  120, 255);
            case 15: // vegetation
                return _scolor3(0,   175, 0);
            case 16: // trunk
                return _scolor3(0,   60,  135);
            case 17: // terrain
                return _scolor3(80,  240, 150);
            case 18: // pole
                return _scolor3(150, 240, 1);
            case 19: // traffic-sign
                return _scolor3(0,   0,   255);
            }
        }
    };

    struct NCLT
    {
        static constexpr size_t NumClass = 14;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0: // ignore
                return _color_ignore;
            case 1: // water
                return _scolor3(30,  144, 250);
            case 2: // road
                return _scolor3(250, 250, 250);
            case 3: // sidewalk
                return _scolor3(128, 64,  128);
            case 4: // terrain
                return _scolor3(128, 128, 0  );
            case 5: // building
                return _scolor3(250, 128, 0  );
            case 6: // vegetation
                return _scolor3(107, 142, 35 );
            case 7: // car
                return _scolor3(0,   0,   142);
            case 8: // person
                return _scolor3(220, 20,  60 );
            case 9: // bike
                return _scolor3(119, 11,  32 );
            case 10: // pole
                return _scolor3(192, 192, 192);
            case 11: // stair
                return _scolor3(123, 104, 238);
            case 12: // traffic sign
                return _scolor3(250, 250, 0  );
            case 13: // sky
                return _scolor3(135, 206, 235);
            }
        }
    };

    /*
     * Label from Ros, German, et al. "Vision-based offline-online perception
     * paradigm for autonomous driving." 2015 IEEE Winter Conference on
     * Applications of Computer Vision. IEEE, 2015.
     */
    struct KITTI
    {
        static constexpr size_t NumClass = 12;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0: // ignore
                return _color_ignore;
            case 1: // building
                return _scolor3(128, 0,   0  );
            case 2: // sky
                return _scolor3(128, 128, 128);
            case 3: // road
                return _scolor3(128, 64,  128);
            case 4: // vegetation
                return _scolor3(128, 128, 0  );
            case 5: // sidewalk
                return _scolor3(0,   0,   192);
            case 6: // car
                return _scolor3(64,  0,   128);
            case 7: // pedestrian
                return _scolor3(64,  64,  0  );
            case 8: // cyclist
                return _scolor3(0,   128, 192);
            case 9: // signate
                return _scolor3(192, 128, 128);
            case 10: // fense
                return _scolor3(64,  64,  128);
            case 11: // pole
                return _scolor3(192, 192, 128);
            }
        }
    };

    struct Nuscenes
    {
        static constexpr size_t NumClass = 17;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0: // ignore
                return _color_ignore;
            case 1: // barrier
                return _scolor3(112, 128, 144);
            case 2: // bicycle
                return _scolor3(220, 20,  60);
            case 3: // bus
                return _scolor3(255, 127, 80);
            case 4: // car
                return _scolor3(255, 158, 0);
            case 5: // construction_vehicle
                return _scolor3(233, 150, 70);
            case 6: // motorcycle
                return _scolor3(255, 61,  99);
            case 7: // pedestrian
                return _scolor3(0,   0,   230);
            case 8: // traffic_cone
                return _scolor3(47,  79,  79);
            case 9: // trailer
                return _scolor3(255, 140, 0);
            case 10: // truck
                return _scolor3(255, 99,  71);
            case 11: // driveable_surface
                return _scolor3(0,   207, 191);
            case 12: // other_flat
                return _scolor3(175, 0,   75);
            case 13: // sidewalk
                return _scolor3(75,  0,   75);
            case 14: // terrain
                return _scolor3(112, 180, 60);
            case 15: // manmade
                return _scolor3(222, 184, 135);
            case 16: // vegetation
                return _scolor3(0,   175, 0);
            }
        }
    };

    // reference: https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
    struct Carla
    {
        static constexpr size_t NumClass = 23;
        static RGB getColor(int semantic)
        {
            assert(0 <= semantic && semantic < NumClass && "Invalid semantic value!");
            switch (semantic)
            {
            default:
            case 0: // Unlabeled
                return _color_ignore;
            case 1: // Building
                return _scolor3(70,  70,  70);
            case 2: // Fence
                return _scolor3(100, 40,  40);
            case 3: // Other
                return _scolor3(55,  90,  80);
            case 4: // Pedestrian
                return _scolor3(220, 20,  60);
            case 5: // Pole
                return _scolor3(153, 153, 153);
            case 6: // RoadLine
                return _scolor3(157, 234, 50);
            case 7: // Road
                return _scolor3(128, 64,  128);
            case 8: // SideWalk
                return _scolor3(244, 64,  128);
            case 9: // Vegetation
                return _scolor3(107, 142, 35);
            case 10: // Vehicles
                return _scolor3(0,   0,   142);
            case 11: // Wall
                return _scolor3(102, 102, 156);
            case 12: // TrafficSign
                return _scolor3(175, 0,   75);
            case 13: // Sky
                return _scolor3(70,  130, 180);
            case 14: // Ground
                return _scolor3(81,  0,   81);
            case 15: // Bridge
                return _scolor3(150, 100, 100);
            case 16: // RailTrack
                return _scolor3(230, 150, 140);
            case 17: // GuardRail
                return _scolor3(180, 165, 180);
            case 18: // TrafficLight
                return _scolor3(250, 170, 30);
            case 19: // Static
                return _scolor3(110, 190, 160);
            case 20: // Dynamic
                return _scolor3(170, 120, 50);
            case 21: // Water
                return _scolor3(45,  60,  150);
            case 22: // Terrain
                return _scolor3(145, 170, 100);
            }
        }
    };
}