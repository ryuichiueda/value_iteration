#include "include/Valueiterator_GoogleTest.h"

using std::ifstream;
using std::ofstream;

Valueiterator_GoogleTest::Valueiterator_GoogleTest() : 
                        _test_num(0), 
                        _return(0) 
                        {};

Valueiterator_GoogleTest::~Valueiterator_GoogleTest() {};

gtest_diff::gtest_diff() :
        _private_nh("~"),
        _map_path("0")
{
    _private_nh.getParam("/test_value_iterator/map_path", _map_path);
};

gtest_diff::~gtest_diff(){};

void gtest_diff::gtest_file_diff2(string &file_path1, string &file_path2)
{
    _return = 0;

    string str_file1, str_file_diff2_sum1, str_file2, str_file_diff2_sum2;
    ifstream file_diff2_1(file_path1);
    ifstream file_diff2_2(file_path2);
    
    //ファイルオープンチェック
    ASSERT_EQ(false, file_diff2_1.fail());
    ASSERT_EQ(false, file_diff2_2.fail());
    
    while(getline(file_diff2_1, str_file_diff2_sum1))
        str_file_diff2_sum1 = str_file_diff2_sum1 + str_file1;

    while(getline(file_diff2_2, str_file_diff2_sum2))
        str_file_diff2_sum2 = str_file_diff2_sum2 + str_file2;
    
    /*For debugging/*
    ofstream file1_w("/tmp/test1.pbm");
    ofstream file2_w("/tmp/test2.pbm");

    file1_w << str_file_diff2_sum1 << endl;
    file2_w << str_file_diff2_sum2 << endl;
    */

   //ファイルの差分比較
    ASSERT_EQ(str_file_diff2_sum1, str_file_diff2_sum2);
    
    _return = 1;
    
};