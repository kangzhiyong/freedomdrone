#pragma once
#include <cmath>
#include <ctime>
using namespace std;

#include "free_point.hpp"

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif // !M_PI

#define K0  0.9996

#define E  0.00669438
#define E2  (E * E)
#define E3  (E2 * E)
#define E_P2  (E / (1.0 - E))

#define SQRT_E  (sqrt(1 - E))
#define _E  ((1 - SQRT_E) / (1 + SQRT_E))
#define _E2  (_E * _E)
#define _E3  (_E2 * _E)
#define _E4  (_E3 * _E)
#define _E5  (_E4 * _E)

#define M1  ((1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256))
#define M2  ((3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024))
#define M3  ((15 * E2 / 256 + 45 * E3 / 1024))
#define M4  ((35 * E3 / 3072))

#define P2  ((3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5))
#define P3  ((21. / 16 * _E2 - 55. / 32 * _E4))
#define P4  ((151. / 96 * _E3 - 417. / 128 * _E5))
#define P5  ((1097. / 512 * _E4))

#define R  6378137

#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define CONSTANTS_RADIUS_OF_EARTH 6371000 // meters (m)

static const float epsilon = std::numeric_limits<float>::epsilon();

class ZoneNode
{
public:
    int lat_min;
    char zone_letter;
    ZoneNode(int n, char l)
    {
        lat_min = n;
        zone_letter = l;
    }
};

static vector< ZoneNode > ZONE_LETTERS = {
                                            ZoneNode(84, ' '), ZoneNode(72, 'X'), ZoneNode(64, 'W'), ZoneNode(56, 'V'), ZoneNode(48, 'U'), ZoneNode(40, 'T'),
                                            ZoneNode(32, 'S'), ZoneNode(24, 'R'), ZoneNode(16, 'Q'), ZoneNode(8, 'P'), ZoneNode(0, 'N'), ZoneNode(-8, 'M'), ZoneNode(-16, 'L'),
                                            ZoneNode(-24, 'K'), ZoneNode(-32, 'J'), ZoneNode(-40, 'H'), ZoneNode(-48, 'G'), ZoneNode(-56, 'F'), ZoneNode(-64, 'E'),
                                            ZoneNode(-72, 'D'), ZoneNode(-80, 'C') };

inline float norm(float x0, float y0, float z0, float x1, float y1, float z1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2));
}

inline float norm(point3D p, int dimension=2)
{
	if (dimension == 2)
	{
		return sqrt(pow(p[0], 2) + pow(p[1], 2));
	}
	else
	{
		return sqrt(pow(p[0], 2) + pow(p[1], 2) + pow(p[2], 2));
	}
}

static int zone_number_to_central_longitude(int zone_number)
{
    return (zone_number - 1) * 6 - 180 + 3;
}

static char latitude_to_zone_letter(double latitude)
{
    for (size_t i = 0; i < ZONE_LETTERS.size(); i++)
    {
        if (latitude >= ZONE_LETTERS[i].lat_min)
        {
            return ZONE_LETTERS[i].zone_letter;
        }
    }
    return ' ';
}

static int latlon_to_zone_number(double latitude, double longitude)
{
    if (56 <= latitude && latitude <= 64 && 3 <= longitude && longitude <= 12)
    {
        return 32;
    }

    if (72 <= latitude && latitude <= 84 && longitude >= 0)
    {
        if (longitude <= 9)
        {
            return 31;
        }
        else if (longitude <= 21)
        {
            return 33;
        }
        else if (longitude <= 33)
        {
            return 35;
        }
        else if (longitude <= 42)
        {
            return 37;
        }
    }
    return int((longitude + 180) / 6) + 1;
}

static void to_latlon(double easting, double northing, int zone_number, char zone_letter, bool northern, double& east, double& north)
{
    if (zone_letter == ' ' && !northern)
    {
        cout << "either zone_letter or northern needs to be set" << endl;
        return;
    }
    else if (zone_letter != ' ' && northern )
    {
        cout << "set either zone_letter or northern, but not both" << endl;
        return;
    }

    if (! (100000 <= easting && easting < 1000000))
    {
        cout << "easting out of range (must be between 100.000 m and 999.999 m)" << endl;
        return;
    }
    if (! (0 <= northing && northing <= 10000000))
    {
        cout << "northing out of range (must be between 0 m and 10.000.000 m)" << endl;
        return;
    }
    if (! (1 <= zone_number && zone_number <= 60))
    {
        cout << "zone number out of range (must be between 1 and 60)" << endl;
        return;
    }

    if (zone_letter != ' ')
    {
        zone_letter = toupper(zone_letter);

        if (! ('C' <= zone_letter && zone_letter <= 'X') || zone_letter  == 'I' || zone_letter == 'O')
        {
            cout << "zone letter out of range (must be between C and X)" << endl;
            return;
        }

        northern = (zone_letter >= 'N');
    }

    double x = easting - 500000;
    double y = northing;

    if (!northern)
    {
        y -= 10000000;
    }

    double m = y / K0;
    double mu = m / (R * M1);

    double p_rad = (mu +
        P2 * sin(2 * mu) +
        P3 * sin(4 * mu) +
        P4 * sin(6 * mu) +
        P5 * sin(8 * mu));

    double p_sin = sin(p_rad);
    double p_sin2 = p_sin * p_sin;

    double p_cos = cos(p_rad);

    double p_tan = p_sin / p_cos;
    double p_tan2 = p_tan * p_tan;
    double p_tan4 = p_tan2 * p_tan2;

    double ep_sin = 1 - E * p_sin2;
    double ep_sin_sqrt = sqrt(1 - E * p_sin2);

    double n = R / ep_sin_sqrt;
    double r = (1 - E) / ep_sin;

    double c = _E * (p_cos, 2);
    double c2 = c * c;

    double d = x / (n * K0);
    double d2 = d * d;
    double d3 = d2 * d;
    double d4 = d3 * d;
    double d5 = d4 * d;
    double d6 = d5 * d;

    double latitude = (p_rad - (p_tan / r) *
                        (d2 / 2 - d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * E_P2)) +
                        d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * E_P2 - 3 * c2));

    double longitude = (d -
                        d3 / 6 * (1 + 2 * p_tan2 + c) +
                        d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * E_P2 + 24 * p_tan4)) / p_cos;

    east = latitude * M_RAD_TO_DEG;
    north = longitude* M_RAD_TO_DEG + zone_number_to_central_longitude(zone_number);
}

static void from_latlon(double latitude, double longitude, int force_zone_number, double& easting, double& northing, int& zone_number, char& zone_letter)
{
    if (!(- 80.0 <= latitude && latitude <= 84.0))
    {
        cout << "latitude out of range (must be between 80 deg S and 84 deg N)" << endl;
        return;
    }
    if (!(- 180.0 <= longitude && longitude <= 180.0))
    {
        cout << "northing out of range (must be between 180 deg W and 180 deg E)" << endl;
        return;
    }

    double lat_rad = latitude * M_DEG_TO_RAD;
    double lat_sin = sin(lat_rad);
    double lat_cos = cos(lat_rad);

    double lat_tan = lat_sin / lat_cos;
    double lat_tan2 = lat_tan * lat_tan;
    double lat_tan4 = lat_tan2 * lat_tan2;

    if (force_zone_number == 0)
    {
        zone_number = latlon_to_zone_number(latitude, longitude);
    }
    else
    {
        zone_number = force_zone_number;
    }

    zone_letter = latitude_to_zone_letter(latitude);

    double lon_rad = longitude * M_DEG_TO_RAD;
    double central_lon = zone_number_to_central_longitude(zone_number);
    double central_lon_rad = central_lon * M_DEG_TO_RAD;

    double n = R / sqrt(1 - E * pow(lat_sin, 2));
    double c = E_P2 * (lat_cos, 2);

    double a = lat_cos * (lon_rad - central_lon_rad);
    double a2 = a * a;
    double a3 = a2 * a;
    double a4 = a3 * a;
    double a5 = a4 * a;
    double a6 = a5 * a;

    double m = R * (M1 * lat_rad -
                    M2 * sin(2 * lat_rad) +
                    M3 * sin(4 * lat_rad) -
                    M4 * sin(6 * lat_rad));

    easting = K0 * n * (a +
                        a3 / 6 * (1 - lat_tan2 + c) +
                        a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000;

    northing = K0 * (m + n * lat_tan * (a2 / 2 +
                    a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * pow(c, 2)) +
                    a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));

    if (latitude < 0)
    {
        northing += 10000000;
    }
}

template<typename coordinate_type>
coordinate_type global_to_local(coordinate_type global_position, coordinate_type global_home)
{
    /*
        Convert a global position(lon, lat, up) to a local position(north, east, down) relative to the home position.
        Returns:
        numpy array of the local position[north, east, down]
    */
    double east_home, north_home, east, north;
    int zone_n;
    char zone_l;
    from_latlon(global_home[1], global_home[0], 0, east_home, north_home, zone_n, zone_l);
    from_latlon(global_position[1], global_position[0], 0, east, north, zone_n, zone_l);

    return { north - north_home, east - east_home, -(global_position[2] - global_home[2]) };
}

template<typename coordinate_type>
coordinate_type local_to_global(coordinate_type local_position, coordinate_type global_home) {
    /*
        Convert a local position(north, east, down) relative to the home position to a global position(lon, lat, up)
        Returns:
        numpy array of the global position[longitude, latitude, altitude]
     */
    double east_home, north_home, lat, lon;
    int zone_number;
    char zone_letter;
    from_latlon(global_home[1], global_home[0], 0, east_home, north_home, zone_number, zone_letter);
    to_latlon(east_home + local_position[1], north_home + local_position[0], zone_number, zone_letter, false, lat, lon);

    return { lon, lat, -(local_position[2] - global_home[2]) };
}

template<typename coordinate_type>
static int clip(coordinate_type data, int d_min, int d_max)
{
    coordinate_type tmp = data;
    if (data < 0)
    {
        tmp = 0;
    }
    else if (data > d_max)
    {
        tmp = d_max;
    }
    return int(tmp);
}

template<typename coordinate_type>
static vector<coordinate_type> uniform(coordinate_type min, coordinate_type max, int num)
{
    vector<coordinate_type> d;
    std::mt19937 generator((std::random_device()()));
    std::uniform_int_distribution<coordinate_type> distribution(min, max);
    
    auto dice= std::bind(distribution,generator);
    for (int i = 0; i < num; i++)
    {
        d.push_back(dice());
    }
    return d;
}
