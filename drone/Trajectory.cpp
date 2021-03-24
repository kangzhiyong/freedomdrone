#include "Trajectory.h"

using namespace SLR;

Trajectory::Trajectory()
{
	_log_file = NULL;
}

Trajectory::Trajectory(const string& filename)
{
	ReadFile(filename);
}

Trajectory::~Trajectory()
{
	// Close any open logging files
	if (_log_file)
	{
		fclose(_log_file);
	}
}

bool Trajectory::ReadFile(const string& filename)
{
	traj.clear();

	FILE* f = fopen(filename.c_str(), "r");
	if (!f)
	{
		return false;
	}

	char buf[512];
	buf[511] = 0; // null char

	// read line by line...
	while (fgets(buf, 510, f))
	{
		string s(buf);

		ParseLine(filename, s);
	}

	fclose(f);

	return true;
}

void Trajectory::ParseLine(const string& filename, const string& s)
{
	std::size_t firstNonWS = s.find_first_not_of("\n\t ");

	// Ignore comments
	if (firstNonWS == std::string::npos || s[firstNonWS] == '#' || firstNonWS == '/')
	{
		return;
	}

	TrajectoryPoint traj_pt;

	V3F ypr; // Helper variable to read in yaw, pitch and roll
	sscanf(s.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &traj_pt.time,
		&traj_pt.position[0], &traj_pt.position[1], &traj_pt.position[2],
		&traj_pt.velocity[0], &traj_pt.velocity[1], &traj_pt.velocity[2],
		&ypr[0], &ypr[1], &ypr[2], &traj_pt.omega[0], &traj_pt.omega[1], &traj_pt.omega[2]);

	// Convert yaw, pitch, and roll to an attitude quaternion
	traj_pt.attitude = Quaternion<float>::FromEulerYPR(ypr[0], ypr[1], ypr[2]);

	// Add the trajectory point to the vector of all trajectory points
	traj.push_back(traj_pt);
}

void Trajectory::Clear()
{
	_curTrajPoint = 0;
	traj.clear();

	// close and reopen the log file
	if (_log_file)
	{
		fclose(_log_file);
		_log_file = nullptr;
	}

	if (!_log_filename.empty())
	{
		_log_file = fopen(_log_filename.c_str(), "w");
	}
}

void Trajectory::SetLogFile(const string& filename)
{
	_log_filename = filename;

	// Close any file that might have been open and open the new file
	if (_log_file)
	{
		fclose(_log_file);

		if (_log_filename != "")
		{
			_log_file = fopen(_log_filename.c_str(), "w");
		}
	}
}

void Trajectory::AddTrajectoryPoint(TrajectoryPoint traj_pt)
{
	traj.push_back(traj_pt);

	// If there is a log file, write the point to file
	if (_log_file)
	{
		WriteTrajectoryPointToFile(_log_file, traj_pt);
	}
}

TrajectoryPoint Trajectory::NextTrajectoryPoint(float time)
{
	if (traj.empty()) return TrajectoryPoint();

	// Loop through the trajectory vector and get the next trajectory point
	for (int i = (int)traj.size() - 1; i >= 0; i--)
	{
		if (traj.at(i).time < time)
		{
			_curTrajPoint = i;
			// interpolation
			if (i == (int)traj.size() - 1)
			{
				// we're at the end of the trajectory
				return traj.at(i);
			}
			else
			{
				float dt = traj.at(i + 1).time - traj.at(i).time;
				float alpha = (time - traj.at(i).time) / dt;
				float beta = 1.f - alpha;
				TrajectoryPoint ret;
				ret.position = traj.at(i).position * beta + traj.at(i + 1).position * alpha;
				ret.velocity = traj.at(i).velocity * beta + traj.at(i + 1).velocity * alpha;
				ret.accel = traj.at(i).accel * beta + traj.at(i + 1).accel * alpha;
				ret.omega = traj.at(i).omega * beta + traj.at(i + 1).omega * alpha;
				ret.attitude = traj.at(i).attitude.Interpolate_SLERP(traj.at(i + 1).attitude, alpha);
				ret.time = time;
				return ret;
			}

			// no interpolation
			return traj.at(i);
		}
	}

	// if requested 0 or negative time
	_curTrajPoint = 0;
	return traj[0];
}

void Trajectory::WriteTrajectoryPointToFile(FILE* f, TrajectoryPoint traj_pt)
{
	if (!f)
	{
		return;
	}

	// Write the trajectory point to file
	V3D ypr = traj_pt.attitude.ToEulerYPR();
	fprintf(f, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", (double)traj_pt.time, \
		(double)traj_pt.position[0], (double)traj_pt.position[1], (double)traj_pt.position[2],
		(double)traj_pt.velocity[0], (double)traj_pt.velocity[1], (double)traj_pt.velocity[2],
		ypr[0], ypr[1], ypr[2], (double)traj_pt.omega[0],
		(double)traj_pt.omega[1], (double)traj_pt.omega[2]);

	// Flush to file
	fflush(f);
}
