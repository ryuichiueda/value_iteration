#include "ValueIterator.h"
#include <thread>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

SweepWorkerStatus::SweepWorkerStatus()
{
	_finished = false;
	_sweep_step = 0;
	_delta = ValueIterator::_max_cost;
}

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_it = theta;
	_cost = ValueIterator::_max_cost;
	_free = (map_value == 0);
	_final_state = false;
}

Action::Action(string name, double fw, double rot)
{
	_name = name;

	_delta_fw = fw;
	_delta_rot = rot;

	//_delta_fw_stdev = fabs(fw)*0.1;
	//_delta_rot_stdev = fabs(rot)*0.1;
}

StateTransition::StateTransition(int dix, int diy, int dit, int prob)
{
	_dix = dix;
	_diy = diy;
	_dit = dit;
	_prob = prob;
}

string StateTransition::to_string(void)
{
	return "dix:" + std::to_string(_dix) + " diy:" + std::to_string(_diy) 
		+ " dit:" + std::to_string(_dit) + " prob:" + std::to_string(_prob);
}

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
							:_pnh("~")
{
	getParameters();
	_image_pub = _pnh.advertise<sensor_msgs::Image>(_imageTopic, 10);

	_cell_x_num = map.info.width;
	_cell_y_num = map.info.height;
	_cell_t_num = 60; //6[deg]刻みでとりあえず固定

	_cell_x_width = map.info.resolution;
	_cell_y_width = map.info.resolution;
	_cell_t_width = 360/_cell_t_num;

	_center_state_ix = _cell_x_num/2;
	_center_state_iy = _cell_y_num/2;

	_final_state_x = 0.0;
	_final_state_y = 0.0;
	_final_state_width = 0.5;

	//_delta = _max_cost;

	for(int y=0; y<_cell_y_num; y++)
		for(int x=0; x<_cell_x_num; x++)
			for(int t=0; t<_cell_t_num; t++)
				_states.push_back(State(x, y, t, map.data[y*_cell_x_num + x]));

	setStateValues();

	imagePublisher();
	setAction();
	setStateTransition();
}

/* デフォルトのアクションの設定 */
void ValueIterator::setAction(void)
{
	_actions.push_back(Action("forward", 0.1, 0.0));
	_actions.push_back(Action("right", 0.0, -10.0));
	_actions.push_back(Action("left", 0.0, 10.0));
}

void ValueIterator::setStateTransition(void)
{
	vector<StateTransition> theta_state_transitions;
	for(auto &a : _actions)
		for(int t=0; t<_cell_t_num; t++)
			a._state_transitions.push_back(theta_state_transitions);

	vector<thread> ths;
	for(int t=0; t<_cell_t_num; t++)
		ths.push_back(thread(&ValueIterator::setStateTransitionWorker, this, t));

	for(auto &th : ths)
		th.join();
}

void ValueIterator::toCellPos(double x, double y, double t, int &ix, int &iy, int &it)
{
	ix = (int)floor(fabs(x) / _cell_x_width);
	if(x < 0.0)
		ix = -ix-1;
	iy = (int)floor(fabs(y) / _cell_y_width);
	if(y < 0.0)
		iy = -iy-1;

	it = (int)floor(t / _cell_t_width);
}


void ValueIterator::setStateTransitionWorker(int it)
{
	for(auto &a : _actions)
		setStateTransition(a, it);
}

void ValueIterator::accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t)
{
	double ang = from_t / 180 * 3.141592;
	to_x = from_x + a._delta_fw*cos(ang);
	to_y = from_y + a._delta_fw*sin(ang);
	to_t = from_t + a._delta_rot;
	while(to_t < 0.0)
		to_t += 360.0;
}

void ValueIterator::setStateTransition(Action &a, int it)
{
	double theta_origin = it*_cell_t_width;

	for(int y=0; y<_resolution_y; y++){
		for(int x=0; x<_resolution_x; x++){
			for(int t=0; t<_resolution_t; t++){
				//遷移前の姿勢
				double ox = ((double)x+0.5)*_cell_x_width/_resolution_x;
				double oy = ((double)y+0.5)*_cell_y_width/_resolution_y;
				double ot = ((double)t+0.5)*_cell_t_width/_resolution_t + theta_origin;

				//遷移後の姿勢
				double dx, dy, dt;
				accurateStateTransition(a, ox, oy, ot, dx, dy, dt);
				int dix, diy, dit;
				toCellPos(dx, dy, dt, dix, diy, dit); 

				bool exist = false;
				for(auto &s : a._state_transitions[it]){
					if(s._dix == dix and s._diy == diy and s._dit == dit){
						s._prob++;
						exist = true;
						break;
					}
				}
				if(not exist)
					a._state_transitions[it].push_back(StateTransition(dix, diy, dit, 1));
			}
		}
	}
}

uint64_t ValueIterator::valueIteration(State &s)
{
	if((not s._free) or s._final_state)
		return 0;

	uint64_t min_cost = ValueIterator::_max_cost;
	for(auto a : _actions)
		min_cost = min(actionCost(s, a), min_cost);

	int64_t delta = min_cost - s._cost;
	s._cost = min_cost;

	return delta > 0 ? delta : -delta;
}

void ValueIterator::valueIterationWorker(int times, int id)
{
	_status.insert(make_pair(id, SweepWorkerStatus()));
	cout << "address:" << &_states[0] << endl;

	for(int j=0; j<times; j++){
		_status[id]._sweep_step = j+1;

		uint64_t max_delta = 0;
	
		int start = rand()%_states.size();
		for(int i=start; i<_states.size(); i++)
			max_delta = max(max_delta, valueIteration(_states[i]));
		for(int i=start-1; i>=0; i--)
			max_delta = max(max_delta, valueIteration(_states[i]));
	
		_status[id]._delta = (double)(max_delta >> _prob_base_bit);
		//delta = max_delta;
		//cout << "delta: " << _status[id]._delta << endl;
		if(_status[id]._delta < 0.1)
			break;
	}

	_status[id]._finished = true;
}

int ValueIterator::toIndex(int ix, int iy, int it)
{
	return it + ix*_cell_t_num + iy*(_cell_t_num*_cell_x_num);
}

uint64_t ValueIterator::actionCost(State &s, Action &a)
{
	uint64_t cost = 0;
	for(auto &tran : a._state_transitions[s._it]){
		int ix = s._ix + tran._dix;
		if(ix < 0 or ix >= _cell_x_num)
			return _max_cost;

		int iy = s._iy + tran._diy;
		if(iy < 0 or iy >= _cell_y_num)
			return _max_cost;

		int it = (s._it + tran._dit + _cell_t_num)%_cell_t_num;

		auto &after_s = _states[toIndex(ix, iy, it)];
		if(not after_s._free)
			return _max_cost;

		cost += (after_s._cost>>_prob_base_bit) * tran._prob;
	}

	return cost + _prob_base;
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void){
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << _cell_x_num << " " << _cell_y_num << endl;
	int i = 0;
	while(i<_states.size()){
		ofs << _states[i]._free << " ";
		i += _cell_t_num;
	}

	ofs << flush;
}

void ValueIterator::setStateValues(void)
{
	for(auto &s : _states){
		double x0 = (s._ix - _center_state_ix)*_cell_x_width;
		double y0 = (s._iy - _center_state_iy)*_cell_y_width;
		double x1 = x0 + _cell_x_width;
		double y1 = y0 + _cell_y_width;

		s._final_state = fabs(x0 - _final_state_x) < _final_state_width
			       && fabs(y0 - _final_state_y) < _final_state_width
			       && fabs(x1 - _final_state_y) < _final_state_width 
			       && fabs(y1 - _final_state_y) < _final_state_width
			       && s._free;
	}

	for(auto &s : _states){
		if(s._final_state)
			s._cost = 0;
		else
			s._cost = _max_cost;
	}
}

// void ValueIterator::outputValuePgmMap(void)
// {
// 	for(int t=0; t<_cell_t_num; t++){
// 		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");

// 		ofs << "P2" << endl;
// 		ofs << _cell_x_num << " " << _cell_y_num << " 255" << endl;
// 		int i = t;
// 		while(i<_states.size()){
// 			uint64_t v = _states[i]._cost >> _prob_base_bit;
// 			if(_states[i]._free and v <= 255)
// 				ofs << 255 - v << " ";
// 			else
// 				ofs << 0 << " ";

// 			i += _cell_t_num;
// 		}
	
// 		ofs << flush;
// 	}
// }

bool ValueIterator::getParameters(void)
{
	_pnh.getParam("image_path", _imagePath);
	_pnh.getParam("image_topic", _imageTopic);
	return true;
}

void ValueIterator::imagePublisher(void)
{
	//ofstreamではなくvectorを検討中
	for(int t=0; t<_cell_t_num; t++){
		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");

		ofs << "P2" << endl;
		ofs << _cell_x_num << " " << _cell_y_num << " 255" << endl;
		int i = t;
		while(i<_states.size()){
			uint64_t v = _states[i]._cost >> _prob_base_bit;
			if(_states[i]._free and v <= 255)
				ofs << 255 - v << " ";
			else
				ofs << 0 << " ";

			i += _cell_t_num;
		}
		
		ofs << flush;
		if(t == 0)
		{
			cv::Mat	image;
			image = cv::imread(_imagePath);

			std::vector<int> param = std::vector<int>(2);
			param[0] = cv::IMWRITE_PNG_COMPRESSION;
			param[1] = 9;//default(3)  0-9　最高設定

			std::vector<uchar>buf;
			cv::imencode(".png", image, buf, param);
			cv::Mat im_decode = cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR);

			cv::Mat source = im_decode;
			cv::Mat alpha_image = cv::Mat(source.size(), CV_8UC3);
			cv::cvtColor(source, alpha_image, CV_RGB2RGBA);
			
			for (int y = 0; y < alpha_image.rows; ++y) {
				for (int x = 0; x < alpha_image.cols; ++x) {
					cv::Vec4b px = alpha_image.at<cv::Vec4b>(x, y);
					if (px[0] + px[1] + px[2] == 0) {
						px[3] = 0;
						alpha_image.at<cv::Vec4b>(x, y) = px;
					}
				}
			}
					
			std_msgs::Header header; // empty header
			header.frame_id = "map_grid";
			header.stamp = ros::Time::now();
			
			cv_bridge::CvImage img_bridge;
			img_bridge = cv_bridge::CvImage(header, "rgba8", alpha_image);
			
			sensor_msgs::Image msg;
			img_bridge.toImageMsg(msg);
			_image_pub.publish(msg);
		}
	}
}
