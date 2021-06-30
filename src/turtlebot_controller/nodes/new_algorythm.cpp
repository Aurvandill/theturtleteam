#include "ros/ros.h"
#include <cstdlib>
#include "turtlebot_controller/turtle_srvs.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

static int pos_row;
static int pos_column;

static int exit_row;
static int exit_column;

static int tmp = 0;

static bool visited[384][384];
static std::string grid[384][384];

std::string path ="";

void findway(int pos_row, int pos_column, std::string grid[384][384], int origin_x, int origin_y);

void fillFlood(std::string grid[384][384], int exit_row, int exit_column);

void map_subscriber(const nav_msgs::OccupancyGrid& msg)
{
    int height = 384;
    int width = 384;

    int origin_x = msg.info.origin.position.x;
    int origin_y = msg.info.origin.position.y;

    int count = 0;
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            int8_t gridentry = msg.data[count];
            std::string fill_element = std::to_string(gridentry);
            grid[j][i] = fill_element;
            count++;
        }
    }

    fillFlood(grid, exit_row, exit_column);
    findway(pos_row, pos_column, grid, origin_x, origin_y);

    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<turtlebot_controller::turtle_srvs>("set_marker_service");
    turtlebot_controller::turtle_srvs data;
    path = path.substr(0, path.length() - 1);
    ROS_INFO("%s",path.c_str());
    data.request.data = path;
    client.call(data);
}

void fillFlood(std::string grid[384][384], int exit_row, int exit_column) 
{
    ROS_INFO("Entered Fill Flood");
    grid[exit_row][exit_column]="1";
    for (int j=0;j<384;j++) {
        for (int i=0;i<384;i++) {
           if(grid[j][i]=="100"){
               grid[j][i]="w";
               if (grid[j][i-1]=="0"){
                   grid[j][i-1]="w";
               }
               if (grid[j+1][i]=="0"){
                   grid[j+1][i]="w";
               }
               if(grid[j][i+1]=="0"){
                   grid[j][i+1]="w";
               }
               if(grid[j-1][i]=="0"){
                   grid[j-1][i]="w";
               }
               if(grid[j+1][i+1]=="0"){
                   grid[j+1][i+1]="w";
               }
               if(grid[j+1][i-1]=="0"){
                   grid[j+1][i-1]="w";
               }
               if(grid[j-1][i-1]=="0"){
                   grid[j-1][i-1]="w";
               }
               if(grid[j-1][i+1]=="0"){
                   grid[j-1][i+1]="w";
               }
               if(grid[j+2][i]=="0"){
                   grid[j+2][i]="w";

               }
               //Cover C-10
               if(grid[j+2][i-1]=="0"){
                   grid[j+2][i-1]="w";

               }
               //Cover C-11
               if(grid[j+2][i-2]=="0"){
                   grid[j+2][i-2]="w";

               }

               //Cover C-12
               if(grid[j+1][i-2]=="0"){
                   grid[j+1][i-2]="w";

               }

               //Cover C-13
               if(grid[j][i-2]=="0"){
                   grid[j][i-2]="w";

               }

               //Cover C-14
               if(grid[j-1][i-2]=="0"){
                   grid[j-1][i-2]="w";

               }

               //Cover C-15
               if(grid[j-2][i-2]=="0"){
                   grid[j-2][i-2]="w";

               }


               //Cover C-16
               if(grid[j-2][i-1]=="0"){
                   grid[j-2][i-1]="w";

               }

               //Cover C-17
               if(grid[j-2][i]=="0"){
                   grid[j-2][i]="w";

               }
               //Cover C-18
               if(grid[j-2][i+1]=="0"){
                   grid[j-2][i+1]="w";

               }

               //Cover C-19
               if(grid[j-2][i+2]=="0"){
                   grid[j-2][i+2]="w";

               }

               //Cover C-20
               if(grid[j-1][i+2]=="0"){
                   grid[j-1][i+2]="w";

               }
               //Cover C-21
               if(grid[j][i+2]=="0"){
                   grid[j][i+2]="w";

               }
               //Cover C-22
               if(grid[j+1][i+2]=="0"){
                   grid[j+1][i+2]="w";

               }
               //Cover C-23
               if(grid[j+2][i+2]=="0"){
                   grid[j+2][i+2]="w";

               }
               //Cover C-24
               if(grid[j+2][i+1]=="0"){
                   grid[j+2][i+1]="w";

               }
           }else if(grid[j][i]=="-1"){
               grid[j][i]="w";
           }
        }}


    //while loop bis current pos (pos_row, pos_column) erreicht worden ist:
     int my_value=1;
    while(grid[pos_row][pos_column]=="0")
    {
        //find lowest value(1)
        for (int j=0;j<384;j++) {
            for (int i=0;i<384;i++) {
                if(grid[j][i]==std::to_string(my_value)){
                   // ROS_INFO("exit found");
                    //check if neighboring cells are empty (filled with 0)
                    if (grid[j][i-1]=="0"){ //condition true? value of current cell + 1 is inserted
                        grid[j][i-1]=std::to_string(my_value + 1);
                    }
                    if (grid[j+1][i]=="0"){
                        grid[j+1][i]=std::to_string(my_value + 1);
                    }
                    if(grid[j][i+1]=="0"){
                        grid[j][i+1]=std::to_string(my_value + 1);
                    }
                    if(grid[j-1][i]=="0"){
                        grid[j-1][i]=std::to_string(my_value + 1);
                    }
                }
            }
        }
        //after every value is found, my_value is increaseb by 1 to find next higher
        my_value++;
    }
}

void findway(int pos_row, int pos_column, std::string grid[384][384], int origin_x, int origin_y) 
{
    //add current position to path string
    double coordinate_x= pos_row*0.05-10;
    double coordinate_y= pos_column*0.05-10;

    std::string s_x = std::to_string(coordinate_x);
    std::string s_y = std::to_string(coordinate_y);

    path.append(s_x);
    path.append(",");
    path.append(s_y);
    path.append(";");

    std::string value_pos = grid[pos_row][pos_column];
    //check if exit is reached (contains 1)
    if(value_pos=="1"){return;}

    //check if neighbor is wall
    if(grid[pos_row][pos_column-1]!="w"&&grid[pos_row][pos_column-1]!="0"&& grid[pos_row][pos_column - 1] != ">"){
        //check if neighbor is smaller than current entry
        if(stoi(grid[pos_row][pos_column-1])<stoi(value_pos)){
            grid[pos_row][pos_column]=">";
            findway(pos_row, pos_column-1, grid, origin_x,origin_y);
            return;
        }
    }

    if(grid[pos_row+1][pos_column]!="w"&&grid[pos_row+1][pos_column]!="0"&& grid[pos_row + 1][pos_column] != ">"){
        //check if neighbor is smaller than current entry
        if(stoi(grid[pos_row+1][pos_column])<stoi(value_pos)){
            grid[pos_row][pos_column]=">";
            findway(pos_row+1, pos_column, grid, origin_x,origin_y);
            return;
        }
    }

    if(grid[pos_row][pos_column+1]!="w"&&grid[pos_row][pos_column+1]!="0"&& grid[pos_row][pos_column + 1] != ">"){
        //check if neighbor is smaller than current entry
        if(stoi(grid[pos_row][pos_column+1])<stoi(value_pos)){
            grid[pos_row][pos_column]=">";
            findway(pos_row, pos_column+1, grid, origin_x,origin_y);
            return;
        }
    }

    if(grid[pos_row-1][pos_column]!="w"&&grid[pos_row-1][pos_column]!="0"&& grid[pos_row - 1][pos_column ] != ">"){
        //check if neighbor is smaller than current entry
        if(stoi(grid[pos_row-1][pos_column])<stoi(value_pos)){
            grid[pos_row][pos_column]=">";
            findway(pos_row-1, pos_column, grid, origin_x,origin_y);
            return;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"algorythm_test");
    ros::NodeHandle nodeHandle;

    //Parameter Server Variablen einzetzen / Standard setzen
    if (!nodeHandle.getParam("/start_position_x", pos_row)){
        ROS_ERROR("Could not find Parameter (pos_row). Standard wird gesetzt");
        pos_row = 200;
    }
    if (!nodeHandle.getParam("/start_position_y", pos_column)){
        ROS_ERROR("Could not find Parameter (column_row). Standard wird gesetzt");
        pos_column = 200;
    }
    std::string pos_exit;
    if (!nodeHandle.getParam("/goal", pos_exit)){
        ROS_ERROR("Could not find Parameter (exit). Standard wird gesetzt");
        pos_exit = "y5";
    }

    if(pos_exit.compare("b1") == 0)
    {
        exit_row = 227;
        exit_column = 227;
    }else if(pos_exit.compare("b2") == 0)
    {
        exit_row = 221;
        exit_column = 227;
    }else if(pos_exit.compare("b3") == 0)
    {
        exit_row = 215;
        exit_column = 227;
    }else if(pos_exit.compare("b4") == 0)
    {
        exit_row = 209;
        exit_column = 227;
    }else if(pos_exit.compare("b5") == 0)
    {
        exit_row = 203;
        exit_column = 227;
    }else if(pos_exit.compare("b6") == 0)
    {
        exit_row = 196;
        exit_column = 227;
    }else if(pos_exit.compare("b7") == 0)
    {
        exit_row = 190;
        exit_column = 227;
    }else if(pos_exit.compare("b8") == 0)
    {
        exit_row = 184;
        exit_column = 227;
    }else if(pos_exit.compare("b9") == 0)
    {
        exit_row = 178;
        exit_column = 227;
    }else if(pos_exit.compare("b10") == 0)
    {
        exit_row = 172;
        exit_column = 227;
    }else

    if(pos_exit.compare("r1") == 0)
    {
        exit_row = 227;
        exit_column = 227;
    }else if(pos_exit.compare("r2") == 0)
    {
        exit_row = 227;
        exit_column = 221;
    }else if(pos_exit.compare("r3") == 0)
    {
        exit_row = 227;
        exit_column = 215;
    }else if(pos_exit.compare("r4") == 0)
    {
        exit_row = 227;
        exit_column = 209;
    }else if(pos_exit.compare("r5") == 0)
    {
        exit_row = 227;
        exit_column = 203;
    }else if(pos_exit.compare("r6") == 0)
    {
        exit_row = 227;
        exit_column = 197;
    }else if(pos_exit.compare("r7") == 0)
    {
        exit_row = 227;
        exit_column = 191;
    }else if(pos_exit.compare("r8") == 0)
    {
        exit_row = 227;
        exit_column = 184;
    }else if(pos_exit.compare("r9") == 0)
    {
        exit_row = 227;
        exit_column = 178;
    }else if(pos_exit.compare("r10") == 0)
    {
        exit_row = 227;
        exit_column = 172;
    }else

    if(pos_exit.compare("g1") == 0)
    {
        exit_row = 227;
        exit_column = 172;
    }else if(pos_exit.compare("g2") == 0)
    {
        exit_row = 221;
        exit_column = 172;
    }else if(pos_exit.compare("g3") == 0)
    {
        exit_row = 215;
        exit_column = 172;
    }else if(pos_exit.compare("g4") == 0)
    {
        exit_row = 209;
        exit_column = 172;
    }else if(pos_exit.compare("g5") == 0)
    {
        exit_row = 203;
        exit_column = 172;
    }else if(pos_exit.compare("g6") == 0)
    {
        exit_row = 197;
        exit_column = 172;
    }else if(pos_exit.compare("g7") == 0)
    {
        exit_row = 191;
        exit_column = 172;
    }else if(pos_exit.compare("g8") == 0)
    {
        exit_row = 184;
        exit_column = 172;
    }else if(pos_exit.compare("g9") == 0)
    {
        exit_row = 179;
        exit_column = 172;
    }else if(pos_exit.compare("g10") == 0)
    {
        exit_row = 172;
        exit_column = 172;
    }else

    if(pos_exit.compare("y1") == 0)
    {
        exit_row = 172;
        exit_column = 227;
    }else if(pos_exit.compare("y2") == 0)
    {
        exit_row = 172;
        exit_column = 220;
    }else if(pos_exit.compare("y3") == 0)
    {
        exit_row = 172;
        exit_column = 214;
    }else if(pos_exit.compare("y4") == 0)
    {
        exit_row = 172;
        exit_column = 208;
    }else if(pos_exit.compare("y5") == 0)
    {
        exit_row = 172;
        exit_column = 202;
    }else if(pos_exit.compare("y6") == 0)
    {
        exit_row = 172;
        exit_column = 196;
    }else if(pos_exit.compare("y7") == 0)
    {
        exit_row = 172;
        exit_column = 190;
    }else if(pos_exit.compare("y8") == 0)
    {
        exit_row = 172;
        exit_column = 184;
    }else if(pos_exit.compare("y9") == 0)
    {
        exit_row = 172;
        exit_column = 178;
    }else if(pos_exit.compare("y10") == 0)
    {
        exit_row = 172;
        exit_column = 172;
    }else{
        // if no right value then values from y5 as default
        exit_row = 172;
        exit_column = 202;
    }

    //subscriber to map topic
    ros::Subscriber sub = nodeHandle.subscribe("/map", 10, map_subscriber);
    ros::spin();
}
