#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>

using namespace std;

struct Obstacle {
    string type;

    double TopLeft[2]; 
    double TopRight[2]; 
    double BotLeft[2]; 
    double BotRight[2]; 

    double center[2];
    double radius;
};

struct Point {
    double x;
    double y;
    double proximity;
};


struct Node {
    double x;
    double y;
    double proximity;
    int id;
    int parent;
};

Point newPoint(uniform_real_distribution<double> x_unif, uniform_real_distribution<double> y_unif, default_random_engine& re) {
    double x = x_unif(re);
    double y = y_unif(re);
    
    Point new_point;
    new_point.x = x;
    new_point.y = y;

    return new_point;

};

bool sortByDist(const Node &node_1, const Node &node_2) { return node_1.proximity < node_2.proximity;};

Node closestNode(vector<Node>& tree, Point& this_point) {
    for(int i=0; i<tree.size();i++) {
        double delt_x_sqrd = pow((tree[i].x - this_point.x), 2);
        double delt_y_sqrd = pow((tree[i].y - this_point.y), 2);
        double dist = sqrt(delt_x_sqrd + delt_y_sqrd);
        tree[i].proximity = dist;
    };

    sort(tree.begin(), tree.end(), sortByDist);

    return tree[0];
};

Node takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id) {
    double dir_x = rand_point.x - nearest_node.x;
    double dir_y = rand_point.y - nearest_node.y;
    double dir_x_sqrd = pow(dir_x, 2);
    double dir_y_sqrd = pow(dir_y, 2);
    double magnitude = sqrt(dir_x_sqrd + dir_y_sqrd);

    double unit_x = dir_x/magnitude;
    double unit_y = dir_y/magnitude;

    double step_x = step_size*unit_x;
    double step_y = step_size*unit_y;

    Node new_node;
    new_node.x = nearest_node.x+step_x;
    new_node.y = nearest_node.y+step_y;
    new_node.id = uniq_id;
    new_node.parent = nearest_node.id;

    uniq_id++;

    return new_node;
};

void generateObstacles(vector<Obstacle>& obstacles) {
    Obstacle box;
    box.TopLeft[0] = 1.;
    box.TopLeft[1] = 3.;

    box.TopRight[0] = 2.;
    box.TopRight[1] = 3.;

    box.BotLeft[0] = 1.;
    box.BotLeft[1] = 1.;

    box.BotRight[0] = 2.;
    box.BotRight[1] = 1.;

    obstacles.push_back(box);
};

bool checkPointCollision(vector<Obstacle>& obstacles, Point& possible_point) {

    for(int j=0; j<obstacles.size(); j++) {
        bool in_horz_bounds = possible_point.x >= obstacles[j].TopLeft[0] && possible_point.x <= obstacles[j].TopRight[0];
        bool in_vert_bounds = possible_point.y >= obstacles[j].BotLeft[1] && possible_point.y <= obstacles[j].TopLeft[1];
        if (in_horz_bounds && in_vert_bounds) {
            return true;
        };
    };
    return false;
};

bool checkNodeCollision(vector<Obstacle>& obstacles, Node& possible_node) {

    for(int j=0; j<obstacles.size(); j++) {
        bool in_horz_bounds = possible_node.x >= obstacles[j].TopLeft[0] && possible_node.x <= obstacles[j].TopRight[0];
        bool in_vert_bounds = possible_node.y >= obstacles[j].BotLeft[1] && possible_node.y <= obstacles[j].TopLeft[1];
        if (in_horz_bounds && in_vert_bounds) {
            return true;
        };
    };
    return false;
};

bool foundGoal(Point goal_pt, Node node, double THRESHOLD) {
    double x_dist_sqrd = pow((goal_pt.x-node.x), 2);
    double y_dist_sqrd = pow((goal_pt.y-node.y), 2);

    double dist = sqrt(x_dist_sqrd+y_dist_sqrd);

    if (dist<THRESHOLD) {
        return true;
    };

    return false;
};

Node getNodefromID(int id, vector<Node> Nodes) {
    for (int i=0; i<Nodes.size(); i++) {
        if (Nodes[i].id == id) {
            return Nodes[i];
        };
    };
};

void getPath(vector<Node> Tree, Node last_node) {
    vector<Node> path;
    path.push_back(last_node);
    Node current_node = last_node;
    Node parent_node;

    int counter = 0;
    while (current_node.id != 0) {
        int parent_id = current_node.parent;
        parent_node = getNodefromID(parent_id, Tree);
        path.push_back(parent_node);
        current_node = parent_node;
    };

    for (int m=0; m<path.size(); m++) {
        cout<<" PATH NODE: "<<path[m].id<<" COORDS: "<<"x: "<<path[m].x<<" ,y: "<<path[m].y<<endl;
    };
};

const double START_X = 0.;
const double START_Y = 0.;

const double GOAL_X = 4.;
const double GOAL_Y = 4.;
double THRESHOLD = .15;

const double X_MIN = -5.;
const double X_MAX = 5.;
const double Y_MIN = -5.;
const double Y_MAX = 5.;
uniform_real_distribution<double> x_dist(X_MIN, X_MAX);
uniform_real_distribution<double> y_dist(Y_MIN, Y_MAX);

const double STEP_SIZE = .2;
const int EPSILON = 10;

int unique_id = 1;
int MAX_ITER = 1000;


int main () {
    default_random_engine re;

    vector<Node> Tree;
    vector<Obstacle> Obstacles;

    generateObstacles(Obstacles);

    Point goal_point;
    goal_point.x = GOAL_X;
    goal_point.y = GOAL_Y;

    Node init_node;
    init_node.x = START_X;
    init_node.y = START_Y;
    init_node.id = 0;
    init_node.parent = 0;

    Tree.push_back(init_node);
    
    int i = 0;

    while (i<MAX_ITER) {
        Point rand_pt = newPoint(x_dist, y_dist, re);

        if (i % EPSILON == 0 && i != 0) {
            rand_pt.x = GOAL_X;
            rand_pt.y = GOAL_Y;
        }

        if (checkPointCollision(Obstacles, rand_pt)) {
            while (checkPointCollision(Obstacles, rand_pt)) {
                rand_pt = newPoint(x_dist, y_dist, re);
            };
        };

        Node closest_node = closestNode(Tree, rand_pt);
        Node new_node = takeStep(rand_pt, closest_node, STEP_SIZE, unique_id);
        //cout<<"RAND POINT POS: "<<rand_pt.x<<" "<<rand_pt.y<<" ";
        //cout<<"CLOSEST NODE: "<<closest_node.id<<" CLOSEST NODE POS: "<<closest_node.x<<" "<<closest_node.y<<" ";
        //cout<<"NEW NODE ID: "<<new_node.id<<" NEW NODE POS: "<<new_node.x<<" "<<new_node.y<<endl;

        if (checkNodeCollision(Obstacles, new_node)) {
            i++;
            continue;
            };

        Tree.push_back(new_node);

        if (foundGoal(goal_point, new_node, THRESHOLD)) {
            cout<<"*******GOAL FOUND******"<<endl;
            getPath(Tree, new_node);
            break;
        };

        i++;
    };

    //cout<<"-------NODES IN TREE-------"<<endl;
    //for (int k=0; k<Tree.size(); k++) {
    //    cout<<"NODE ID: "<<Tree[k].id<<" "<<"NODE PARENT: "<<Tree[k].parent<<endl;
    //};


   return 0;
}
