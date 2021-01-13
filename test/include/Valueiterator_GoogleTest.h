#ifndef VALUEITERATOR_GOOGLETEST_
#define VALUEITERATOR_GOOGLETEST_

#include <ros/ros.h>

#include <fstream>

#include <gtest/gtest.h>

using std::string;

class Valueiterator_GoogleTest : public ::testing::Test
{
private:
    
public:
    Valueiterator_GoogleTest();
    ~Valueiterator_GoogleTest();

    unsigned int _test_num;
    bool _return;
};

class gtest_diff : public ::Valueiterator_GoogleTest
{
private:
    
public:
    gtest_diff();
    ~gtest_diff();

    ros::NodeHandle _private_nh;
    string _map_path;

    void gtest_file_diff2(string &file_path1, string &file_path2);
};
#endif