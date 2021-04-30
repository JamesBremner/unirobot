#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

/// edge properties
class cEdge
{
public:
    cEdge()
        : myCost(1)
    {
    }
    int myCost;
};

/// node properties
class cNode
{
public:
    cNode(const std::string &name)
        : myName(name)
    {
    }
    cNode()
        : myName("???")
    {
    }
    std::string myName;
};

/// declaration of boost graph configuration
using namespace boost;
typedef boost::adjacency_list<
    boost::listS,
    boost::vecS,
    boost::directedS,
    cNode,
    cEdge>
    graph_t;

/// declaration of structure holding link input data
struct sBiLink
{
    int src;
    int dst;
    int dir; // allowed orientations of robot on link
    int cost;
};

class cRobot
{
public:
    graph_t myGraph;
    std::vector<int> myPath;

    graph_t ConstructBoostGraph(
        const std::vector<sBiLink> &vForward,
        const std::vector<sBiLink> &vBack,
        const std::vector<int> &vTurn);

    /** Find path
 * @param[in] start starting node info
 * @param[in] goal index of node aiming for
 */
    void Path(
        sBiLink &start,
        int goal);

    std::string linksText();
    std::string pathText();

private:
    /** Find or add node by name
 * @return index of node found
 * 
 * If node by same bname already exists, return its index
 * otherwise add a new node and return its index
 */
    int findoradd(const std::string &name);

    /** Find node by name
 * @return index of node found, -1 if not found
 * 
 */
    int find(const std::string &name);

    /** Add costed link
     * @param[in] s mode index
     * @param[in] d node index
     * @param[in] cost
     */
    void addLink(
        int s, int d,
        int cost);
};

std::vector<std::string> ParseSpaceDelimited(
    const std::string &l)
{
    std::vector<std::string> token;
    std::stringstream sst(l);
    std::string a;
    while (getline(sst, a, ' '))
        token.push_back(a);

    token.erase(
        remove_if(
            token.begin(),
            token.end(),
            [](std::string t) {
                return (t.empty());
            }),
        token.end());

    return token;
}

/// read input
std::vector<sBiLink> read(
    const std::string &fname,
    std::vector<int> &vTurn,
    sBiLink &start,
    int &goal)
{
    std::ifstream inf(fname);
    if (!inf.is_open())
    {
        std::cout << "cannot open " << fname << "\n";
    }
    std::vector<sBiLink> vBiLink;
    vTurn.clear();
    std::string line;
    int src, dst, dir;
    while (std::getline(inf, line))
    {
        std::cout << line << "\n";
        auto token = ParseSpaceDelimited(line);
        if (!token.size())
            continue;
        switch (token[0][0])
        {
        case 'l':
            if (token.size() != 5)
            {
                std::cout << "bad link: " << line << "\n";
                exit(1);
            }
            sBiLink l;
            l.src = atoi(token[1].c_str());
            l.dst = atoi(token[2].c_str());
            l.dir = atoi(token[3].c_str());
            l.cost = atoi(token[4].c_str());
            vBiLink.push_back(l);
            break;

        case 's':
            if (token.size() != 3)
            {
                std::cout << "bad start: " << line << "\n";
                exit(1);
            }
            start.src = atoi(token[1].c_str());
            if (token[2] == "b")
                start.dir = 2;
            else
                start.dir = 1;
            break;

        case 'g':
            if (token.size() != 2)
            {
                std::cout << "bad goal: " << line << "\n";
                exit(1);
            }
            goal = atoi(token[1].c_str());
            break;

        case 't':
            if (token.size() != 5)
            {
                std::cout << "bad turning node: " << line << "\n";
                exit(1);
            }
            vTurn.push_back(atoi(token[1].c_str()));
            break;

        default:
            std::cout << "bad row: " << line << "\n";
            break;
        }
    }
    std::cout << vBiLink.size() << " bidirectional links input\n";
    ;
    return vBiLink;
}

/** Split links according to allowed direction of travel
 * @param[in] vBiLink the input links
 * @param[out] vForward links allowing forward travel
 * @param[out] vBack links allowing backward travel
 * @param[out] vTurn modes where robot can turn around
 */
void split(
    const std::vector<sBiLink> &vBiLink,
    std::vector<sBiLink> &vForward,
    std::vector<sBiLink> &vBack,
    std::vector<int> &vTurn)
{

    for (auto &l : vBiLink)
    {
        switch (l.dir)
        {
        case 0:
            vForward.push_back(l);
            vBack.push_back(l);
            break;

        case 1:
            vForward.push_back(l);
            break;

        case 2:
            vBack.push_back(l);
            break;
        }
    }

    std::cout << "\nForward links\n";
    for (auto &l : vForward)
    {
        std::cout
            << std::to_string(l.src)
            << "f - "
            << std::to_string(l.dst)
            << "f\n";
    }
    std::cout << "\nBack links\n";
    for (auto &l : vBack)
    {
        std::cout
            << std::to_string(l.src)
            << "b - "
            << std::to_string(l.dst)
            << "b\n";
    }
    std::cout << "\nTurning Links\n";
    for (auto &n : vTurn)
    {
        std::cout
            << std::to_string(n)
            << "f - "
            << std::to_string(n)
            << "b\n";
    }
}

/** Find node by name
 * @return index of node found
 * 
 * If node by same bname already exists, return its index
 * otherwise add a new node and return its index
 */
int cRobot::findoradd(const std::string &name)
{
    int n = find(name);
    if (n < 0)
        n = add_vertex(name, myGraph);
    return n;
}

int cRobot::find(const std::string &name)
{
    for (int n = 0; n < num_vertices(myGraph); n++)
    {
        if (myGraph[n].myName == name)
        {
            return n;
        }
    }
    return -1;
}

/// Create boost graph from combination of forward, backward and turning links
graph_t cRobot::ConstructBoostGraph(
    const std::vector<sBiLink> &vForward,
    const std::vector<sBiLink> &vBack,
    const std::vector<int> &vTurn)
{
    for (auto &l : vForward)
    {
        auto s = findoradd(std::to_string(l.src) + "f");
        auto d = findoradd(std::to_string(l.dst) + "f");
        addLink(s, d, l.cost);
    }
    for (auto &l : vBack)
    {
        auto s = findoradd(std::to_string(l.src) + "b");
        auto d = findoradd(std::to_string(l.dst) + "b");
        addLink(s, d, l.cost);
    }
    for (int n : vTurn)
    {
        // add edge begining and ending on node
        // which allows the robot to turn around
        // at zero cost
        auto s = findoradd(std::to_string(n) + "f");
        auto d = findoradd(std::to_string(n) + "b");
        addLink(s, d, 0);
    }

    return myGraph;
}

void cRobot::addLink(
    int s, int d,
    int cost)
{
    graph_t::edge_descriptor e = add_edge(s, d, myGraph).first;
    myGraph[e].myCost = cost;
    e = add_edge(d, s, myGraph).first;
    myGraph[e].myCost = cost;
}

std::string cRobot::linksText()
{
    std::stringstream ss;
    //     for (int n = 0; n < num_vertices(myGraph); n++)
    // {
    //     std::cout << myGraph[n].myName << " ";
    // }
    // ss << "\n";

    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(myGraph); ei != ei_end; ++ei)
    {
        ss << "("
           << myGraph[source(*ei, myGraph)].myName << ","
           << myGraph[target(*ei, myGraph)].myName << ","
           << myGraph[*ei].myCost
           << ") ";
    }
    ss << "\n";
    return ss.str();
}

/** Find path
 * @param[in] G the graph
 * @param[in] start starting node info
 * @param[in] goal index of node aiming for
 */
void cRobot::Path(
    sBiLink &start,
    int goal)
{
    // starting node in boost graph
    std::string startNodeName = std::to_string(start.src);
    if (start.dir == 1)
        startNodeName += "f";
    else
        startNodeName += "b";
    int startNode = find(startNodeName);
    if (startNode < 0)
    {
        std::cout << "start " << startNodeName << " " << startNode << "\n";
        throw std::runtime_error("Bad path start");
    }

    // goal nodes in boost graph
    std::string sgf = std::to_string(goal) + "f";
    std::string sgb = std::to_string(goal) + "b";
    int goalf = find(sgf);
    int goalb = find(sgb);
    if (goalf < 0 && goalb < 0)
        throw std::runtime_error("Bad path goal");

    // run dijkstra algorithm
    std::vector<int> p(num_vertices(myGraph));
    std::vector<int> vDist(num_vertices(myGraph));
    boost::dijkstra_shortest_paths(
        myGraph,
        startNode,
        weight_map(get(&cEdge::myCost, myGraph))
            .predecessor_map(boost::make_iterator_property_map(
                p.begin(), get(boost::vertex_index, myGraph)))
            .distance_map(boost::make_iterator_property_map(
                vDist.begin(), get(boost::vertex_index, myGraph))));

    // choose closest possible goal
    int goalnode = goalf;
    if (goalf < 0 || goalb < 0)
    {
        if (goalf < 0)
            goalnode = goalb;
    }
    else
    {
        if (vDist[goalb] < vDist[goalf])
            goalnode = goalb;
    }

    // pick out path, starting at goal and finishing at start
    myPath.push_back(goalnode);
    int prev = goalnode;
    while (1)
    {
        //std::cout << prev << " " << p[prev] << ", ";
        int next = p[prev];
        myPath.push_back(next);
        if (next == startNode)
            break;
        prev = next;
    }

    // reverse so path goes from start to goal
    std::reverse(myPath.begin(), myPath.end());
}

std::string cRobot::pathText()
{
    std::stringstream ss;
    for (auto n : myPath)
        ss << myGraph[n].myName << " -> ";
    ss << "\n";
    return ss.str();
}

main(int argc, char *argv[])
{
    std::cout << "unirobot\n";

    // check command line
    if (argc != 2)
    {
        std::cout << "usage: unirobot <inputfilename>\n";
        exit(0);
    }

    cRobot Robot;

    // read input file
    sBiLink start;
    int goal;
    std::vector<int> vTurn;
    auto vBiLink = read(
        argv[1],
        vTurn,
        start,
        goal);

    // split graph
    std::vector<sBiLink> vForward;
    std::vector<sBiLink> vBack;
    split(
        vBiLink,
        vForward,
        vBack,
        vTurn);

    // construct boost graph
    auto G = Robot.ConstructBoostGraph(
        vForward,
        vBack,
        vTurn);

    std::cout << "\nCombined graph links\n"
              << Robot.linksText();

    // find path from start to goal
    Robot.Path(
        start,
        goal);

    // display path found
    std::cout << "\nPath: "
              << Robot.pathText();
}